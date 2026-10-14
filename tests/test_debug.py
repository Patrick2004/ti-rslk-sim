"""Debugger server (GDB remote protocol) and engine-rebuild robustness."""
import socket
import time

import pytest

from conftest import run, values


class RspClient:
    """Tiny GDB remote-protocol client that drives the stub in the same thread."""

    def __init__(self, stub, session):
        self.stub, self.s = stub, session
        self.sock = socket.create_connection(("127.0.0.1", stub.port))
        self.sock.setblocking(False)
        self.buf = b""

    def pump(self, sim_step=0.002):
        self.stub.poll()
        if self.stub.connected and not self.stub.halted:
            self.s.run_for(sim_step)
            if self.s.m.debug_halt:
                self.stub.machine_stopped_at_breakpoint()

    def cmd(self, payload, timeout=20.0):
        data = payload.encode()
        self.sock.sendall(b"$" + data + b"#" + f"{sum(data) & 0xFF:02x}".encode())
        end = time.time() + timeout
        while time.time() < end:
            self.pump()
            try:
                self.buf += self.sock.recv(65536)
            except BlockingIOError:
                pass
            self.buf = self.buf.lstrip(b"+")
            if self.buf.startswith(b"$") and b"#" in self.buf:
                i = self.buf.index(b"#")
                if len(self.buf) >= i + 3:
                    reply = self.buf[1:i].decode()
                    self.buf = self.buf[i + 3:]
                    self.sock.sendall(b"+")
                    return reply
        raise TimeoutError(payload)


def test_gdb_breakpoint_step_and_memory(fw):
    from rslk_sim.gdbstub import GdbStub
    from rslk_sim.session import Session
    s = Session(fw("t_uart_systick"), None, log=lambda m: None)
    stub = GdbStub(s, port=0 or _free_port(), log=lambda m: None)
    c = RspClient(stub, s)
    for _ in range(50):
        c.pump()
    assert stub.connected and stub.halted
    assert c.cmd("?") == "S05"
    target = s.prog.symbols["main"]
    assert c.cmd(f"Z0,{target:x},2") == "OK"
    reply = c.cmd("c")                                   # runs until the breakpoint
    assert reply.startswith("T05")
    assert s.m.pc == target
    regs = c.cmd("g")
    pc = int.from_bytes(bytes.fromhex(regs[15 * 8:16 * 8]), "little")
    assert pc == target
    assert c.cmd("s").startswith("T05")                  # single step moves on
    assert s.m.pc != target
    # memory: the vector table's reset entry and a peripheral register (CS->CTL1 still at its
    # reset value 0x33 because main() has not called Clock_Init48MHz yet)
    reset_vec = int.from_bytes(bytes.fromhex(c.cmd("m4,4")), "little")
    assert reset_vec & ~1 == s.prog.symbols["Reset_Handler"]
    ctl1 = int.from_bytes(bytes.fromhex(c.cmd("m40010408,4")), "little")
    assert ctl1 == 0x00000033
    assert c.cmd(f"z0,{target:x},2") == "OK"
    c.cmd("D")


def _free_port():
    with socket.socket() as so:
        so.bind(("127.0.0.1", 0))
        return so.getsockname()[1]


def test_engine_rebuild_is_transparent(fw, monkeypatch):
    import rslk_sim.machine as M
    monkeypatch.setattr(M, "ENGINE_CALLS_LIMIT", 200)    # swap the CPU engine very often
    s, out = run(fw("t_interrupts"), seconds=1.5)
    assert 998 <= values(out, "ta1")[0] <= 1001
    assert values(out, "isr_acc")[0] == 2500 and values(out, "main_ok")[0] == 1
    s, out = run(fw("t_line_follow"), world="line_oval", seconds=8.0)
    assert s.robot.distance > 0.8 and values(out, "lost")[-1] == 0
