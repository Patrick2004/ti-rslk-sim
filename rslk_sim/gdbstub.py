"""GDB remote serial protocol server for the simulated MSP432.

Connect with the arm-none-eabi-gdb that comes with CCS:
    arm-none-eabi-gdb Lab.out -ex "target remote localhost:3333"
or from VS Code (see README).  Breakpoints, single-step, continue, Ctrl-C,
register/memory/variable inspection and 'monitor reset' are supported.
"""
import select
import socket

from unicorn import UcError
from unicorn import arm_const as A

TARGET_XML = """<?xml version="1.0"?>
<!DOCTYPE target SYSTEM "gdb-target.dtd">
<target version="1.0">
  <architecture>arm</architecture>
  <feature name="org.gnu.gdb.arm.m-profile">
    <reg name="r0" bitsize="32"/><reg name="r1" bitsize="32"/><reg name="r2" bitsize="32"/>
    <reg name="r3" bitsize="32"/><reg name="r4" bitsize="32"/><reg name="r5" bitsize="32"/>
    <reg name="r6" bitsize="32"/><reg name="r7" bitsize="32"/><reg name="r8" bitsize="32"/>
    <reg name="r9" bitsize="32"/><reg name="r10" bitsize="32"/><reg name="r11" bitsize="32"/>
    <reg name="r12" bitsize="32"/>
    <reg name="sp" bitsize="32" type="data_ptr"/>
    <reg name="lr" bitsize="32"/>
    <reg name="pc" bitsize="32" type="code_ptr"/>
    <reg name="xpsr" bitsize="32" regnum="25"/>
  </feature>
  <feature name="org.gnu.gdb.arm.vfp">
    <reg name="d0" bitsize="64" type="ieee_double"/><reg name="d1" bitsize="64" type="ieee_double"/>
    <reg name="d2" bitsize="64" type="ieee_double"/><reg name="d3" bitsize="64" type="ieee_double"/>
    <reg name="d4" bitsize="64" type="ieee_double"/><reg name="d5" bitsize="64" type="ieee_double"/>
    <reg name="d6" bitsize="64" type="ieee_double"/><reg name="d7" bitsize="64" type="ieee_double"/>
    <reg name="d8" bitsize="64" type="ieee_double"/><reg name="d9" bitsize="64" type="ieee_double"/>
    <reg name="d10" bitsize="64" type="ieee_double"/><reg name="d11" bitsize="64" type="ieee_double"/>
    <reg name="d12" bitsize="64" type="ieee_double"/><reg name="d13" bitsize="64" type="ieee_double"/>
    <reg name="d14" bitsize="64" type="ieee_double"/><reg name="d15" bitsize="64" type="ieee_double"/>
    <reg name="fpscr" bitsize="32" type="int" group="float"/>
  </feature>
</target>
"""

CORE_REGS = [getattr(A, f"UC_ARM_REG_R{i}") for i in range(13)] + [A.UC_ARM_REG_SP, A.UC_ARM_REG_LR, A.UC_ARM_REG_PC]
# gdb register numbers: 0-15 core, 25 xpsr, 26-41 d0-d15, 42 fpscr (from TARGET_XML numbering)
D_REGS = [getattr(A, f"UC_ARM_REG_D{i}") for i in range(16)]


def _checksum(data):
    return sum(data) & 0xFF


class GdbStub:
    def __init__(self, session, port=3333, log=print):
        self.session = session
        self.log = log
        self.port = port
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(1)
        self.srv.setblocking(False)
        self.conn = None
        self.buf = b""
        self.breakpoints = set()
        self.halted = False              # held by the debugger
        self.waiting_stop = False        # gdb is waiting for a stop reply
        self.stop_signal = 5
        self.no_ack = False
        self._install_machine_hooks()
        session.rebuild_listeners.append(self.machine_rebuilt)
        log(f"GDB server listening on localhost:{port} - connect with: "
            f"arm-none-eabi-gdb <program.out> -ex \"target remote localhost:{port}\"")

    # -- machine integration ---------------------------------------------------
    def _install_machine_hooks(self):
        m = self.session.m
        m.debug_halt = False
        m.debug_skip_bp = None
        for addr in self.breakpoints:
            m.add_breakpoint(addr)

    def machine_rebuilt(self):
        """Call after the session rebuilds its Machine (reset)."""
        self._install_machine_hooks()

    @property
    def m(self):
        return self.session.m

    # -- socket ----------------------------------------------------------------
    @property
    def connected(self):
        return self.conn is not None

    def poll(self):
        """Handle pending GDB traffic.  Call often (every frame / every step)."""
        if self.conn is None:
            try:
                self.conn, addr = self.srv.accept()
                self.conn.setblocking(False)
                self.no_ack = False
                self.halted = True
                self.waiting_stop = False
                self.log(f"debugger connected from {addr[0]} - program paused")
            except (BlockingIOError, OSError):
                return
        try:
            while True:
                r, _, _ = select.select([self.conn], [], [], 0)
                if not r:
                    break
                data = self.conn.recv(65536)
                if not data:
                    self._disconnect()
                    return
                self.buf += data
        except (ConnectionError, OSError):
            self._disconnect()
            return
        self._process()

    def _disconnect(self):
        self.log("debugger disconnected - program continues")
        try:
            self.conn.close()
        except OSError:
            pass
        self.conn = None
        self.halted = False
        self.waiting_stop = False

    def _send_raw(self, b):
        try:
            self.conn.sendall(b)
        except OSError:
            self._disconnect()

    def _send(self, payload):
        if isinstance(payload, str):
            payload = payload.encode("latin-1")
        self._send_raw(b"$" + payload + b"#" + f"{_checksum(payload):02x}".encode())

    def _process(self):
        while self.buf:
            c = self.buf[:1]
            if c in (b"+", b"-"):
                self.buf = self.buf[1:]
                continue
            if c == b"\x03":
                self.buf = self.buf[1:]
                self.request_halt(2)
                continue
            if c != b"$":
                self.buf = self.buf[1:]
                continue
            end = self.buf.find(b"#")
            if end < 0 or len(self.buf) < end + 3:
                return
            payload = self.buf[1:end]
            self.buf = self.buf[end + 3:]
            if not self.no_ack:
                self._send_raw(b"+")
            self._handle(payload.decode("latin-1"))

    # -- run control -----------------------------------------------------------
    def request_halt(self, sig=2):
        if not self.halted:
            self.halted = True
            self._report_stop(sig)

    def machine_stopped_at_breakpoint(self):
        """The machine hit a breakpoint during run_until()."""
        self.m.debug_halt = False
        self.halted = True
        self._report_stop(5)

    def _report_stop(self, sig):
        self.stop_signal = sig
        if self.waiting_stop:
            self.waiting_stop = False
            pc = self.m.pc
            self._send(f"T{sig:02x}0f:{(pc).to_bytes(4, 'little').hex()};thread:1;")
        where = self.m.prog.describe(self.m.pc)
        self.log(f"debugger: paused at {where}")

    def _resume(self):
        m = self.m
        if m.pc in self.breakpoints:
            m.debug_skip_bp = m.pc
        self.halted = False
        self.waiting_stop = True

    def _step(self):
        m = self.m
        m.step_instruction()
        self.waiting_stop = True
        self._report_stop(5)

    # -- registers / memory ------------------------------------------------------
    def _sync_pc(self):
        """Machine keeps PC in m.pc between chunks; keep Unicorn's copy equal."""
        self.m.uc.reg_write(A.UC_ARM_REG_PC, self.m.pc | 1)

    def _reg_bytes(self, n):
        uc = self.m.uc
        if n < 16:
            v = self.m.pc if n == 15 else uc.reg_read(CORE_REGS[n])
            return (v & 0xFFFFFFFF).to_bytes(4, "little")
        if n == 25:
            return (uc.reg_read(A.UC_ARM_REG_XPSR) | (1 << 24)).to_bytes(4, "little")
        if 26 <= n < 42:
            return (uc.reg_read(D_REGS[n - 26]) & (2 ** 64 - 1)).to_bytes(8, "little")
        if n == 42:
            return uc.reg_read(A.UC_ARM_REG_FPSCR).to_bytes(4, "little")
        return b"\x00" * 4

    def _set_reg(self, n, raw):
        uc = self.m.uc
        v = int.from_bytes(raw, "little")
        if n < 16:
            if n == 15:
                self.m.pc = v & ~1
                self._sync_pc()
            else:
                uc.reg_write(CORE_REGS[n], v)
        elif n == 25:
            uc.reg_write(A.UC_ARM_REG_APSR, v & 0xF80F0000)
        elif 26 <= n < 42:
            uc.reg_write(D_REGS[n - 26], v)
        elif n == 42:
            uc.reg_write(A.UC_ARM_REG_FPSCR, v)

    def _all_regs(self):
        out = b"".join(self._reg_bytes(i) for i in range(16))
        return out.hex()

    def _read_mem(self, addr, length):
        m = self.m
        out = bytearray()
        a = addr
        while len(out) < length:
            if 0x40000000 <= a < 0x40020000 or 0xE0000000 <= a < 0xE0100000:
                # peripheral registers: go through the models (like a real debug probe would)
                try:
                    if a >= 0xE0000000:
                        v = m.ppb.read(a - 0xE0000000, 1)
                    else:
                        p = m.pages[(a - 0x40000000) >> 10]
                        v = p.read(a - 0x40000000 - p.base_off, 1)
                except Exception:
                    v = 0
                out.append(v & 0xFF)
                a += 1
                continue
            try:
                chunk = min(length - len(out), 0x1000 - (a & 0xFFF))
                out += m.uc.mem_read(a, chunk)
                a += chunk
            except UcError:
                if not out:
                    return None
                break
        return bytes(out)

    def _write_mem(self, addr, data):
        try:
            self.m.uc.mem_write(addr, data)
            return True
        except UcError:
            return False

    # -- packet dispatch -----------------------------------------------------------
    def _handle(self, p):
        m = self.m
        if p.startswith("qSupported"):
            self._send("PacketSize=4000;qXfer:features:read+;swbreak+;hwbreak+;QStartNoAckMode+")
        elif p == "QStartNoAckMode":
            self._send("OK")
            self.no_ack = True
        elif p.startswith("qXfer:features:read:target.xml:"):
            off, length = (int(x, 16) for x in p.split(":")[-1].split(","))
            data = TARGET_XML[off:off + length]
            self._send(("l" if off + length >= len(TARGET_XML) else "m") + data)
        elif p == "?":
            self._send(f"S{self.stop_signal:02x}")
        elif p.startswith("H"):
            self._send("OK")
        elif p in ("qC",):
            self._send("QC1")
        elif p == "qAttached":
            self._send("1")
        elif p == "qfThreadInfo":
            self._send("m1")
        elif p == "qsThreadInfo":
            self._send("l")
        elif p.startswith("qThreadExtraInfo"):
            self._send(b"main".hex())
        elif p.startswith("T"):
            self._send("OK")
        elif p == "g":
            self._send(self._all_regs())
        elif p.startswith("G"):
            raw = bytes.fromhex(p[1:])
            for i in range(min(16, len(raw) // 4)):
                self._set_reg(i, raw[4 * i:4 * i + 4])
            self._send("OK")
        elif p.startswith("p"):
            self._send(self._reg_bytes(int(p[1:], 16)).hex())
        elif p.startswith("P"):
            n, v = p[1:].split("=")
            self._set_reg(int(n, 16), bytes.fromhex(v))
            self._send("OK")
        elif p.startswith("m"):
            a, ln = (int(x, 16) for x in p[1:].split(","))
            data = self._read_mem(a, ln)
            self._send(data.hex() if data is not None else "E01")
        elif p.startswith("M"):
            head, data = p[1:].split(":")
            a, _ = (int(x, 16) for x in head.split(","))
            self._send("OK" if self._write_mem(a, bytes.fromhex(data)) else "E01")
        elif p[:2] in ("Z0", "Z1"):
            a = int(p.split(",")[1], 16) & ~1
            self.breakpoints.add(a)
            m.add_breakpoint(a)
            self._send("OK")
        elif p[:2] in ("z0", "z1"):
            a = int(p.split(",")[1], 16) & ~1
            self.breakpoints.discard(a)
            m.remove_breakpoint(a)
            self._send("OK")
        elif p == "c" or p.startswith("c"):
            if len(p) > 1:
                m.pc = int(p[1:], 16) & ~1
            self._resume()
        elif p == "s" or p.startswith("s"):
            self._step()
        elif p == "vCont?":
            self._send("vCont;c;C;s;S;t")
        elif p.startswith("vCont;"):
            act = p[6:].split(";")[0]
            if act[0] in "sS":
                self._step()
            elif act[0] in "cC":
                self._resume()
            elif act[0] == "t":
                self.request_halt(2)
        elif p.startswith("qRcmd,"):
            cmd = bytes.fromhex(p[6:]).decode("latin-1").strip()
            self._monitor(cmd)
        elif p == "D" or p.startswith("D;"):
            self._send("OK")
            self._disconnect()
        elif p == "k":
            self._disconnect()
        else:
            self._send("")

    def _monitor(self, cmd):
        def say(text):
            self._send("O" + (text + "\n").encode().hex())
        if cmd in ("reset", "reset halt", "reset init", "system_reset"):
            self.session.reset(keep_pose=True)
            self.machine_rebuilt()
            say(f"program restarted at {self.m.prog.describe(self.m.pc)}")
        elif cmd == "halt":
            self.halted = True
        else:
            say("monitor commands: reset")
        self._send("OK")
