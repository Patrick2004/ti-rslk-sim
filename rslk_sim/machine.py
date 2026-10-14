"""The emulated MSP432P401R: Cortex-M4 core (Unicorn) + peripheral models + run loop."""
import ctypes
import math
import struct
import time

from unicorn import (Uc, UcError, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS,
                     UC_HOOK_MEM_UNMAPPED, UC_HOOK_MEM_PROT, UC_HOOK_INTR, UC_PROT_ALL,
                     UC_PROT_READ, UC_PROT_EXEC, UC_MEM_READ_UNMAPPED, UC_MEM_WRITE_UNMAPPED,
                     UC_MEM_FETCH_UNMAPPED, UC_MEM_WRITE_PROT, UC_MEM_FETCH_PROT)
from unicorn.arm_const import UC_CPU_ARM_CORTEX_M4
from unicorn import arm_const as A

from .fastregs import RegBatch
from .regnames import reg_name
from .sched import Scheduler
from .periph.base import Unmodelled
from .periph.core import NVIC, SysTick, PPB, exception_name, EXC_SYSTICK
from .periph.sysctl import CS, PCM, FLCTL, WDT, REF, RSTCTL
from .periph.gpio import GPIO, PORT_IRQ, INTERRUPT_PORTS
from .periph.timer_a import TimerA, TIMER_PINS
from .periph.timer32 import Timer32, T32_IRQ, T32_INTC
from .periph.adc14 import ADC14, CHANNEL_PINS, IRQ as ADC_IRQ
from .periph.eusci import EUSCI
from .periph.base import Peripheral

FLASH_BASE, FLASH_SIZE = 0x00000000, 0x40000
INFO_BASE, INFO_SIZE = 0x00200000, 0x4000
SRAM_BASE, SRAM_SIZE = 0x20000000, 0x10000
SRAM_CODE_ALIAS = 0x01000000
PERIPH_BASE, PERIPH_SIZE = 0x40000000, 0x20000
BB_PERIPH_BASE, BB_PERIPH_SIZE = 0x42000000, PERIPH_SIZE * 32
BB_SRAM_BASE, BB_SRAM_SIZE = 0x22000000, SRAM_SIZE * 32
PPB_BASE, PPB_SIZE = 0xE0000000, 0x100000
TRAMP = 0x0FF00000            # helper page: a 'bx lr' stub used to return from skipped delay calls
STOP_ADDR = TRAMP + 0x800     # 'until' address for emu_start; never executed
EXC_RETURN = 0xFFFFFFF9       # real Cortex-M EXC_RETURN (thread mode, MSP, basic frame)
EXCP_EXCEPTION_EXIT = 8       # Unicorn/QEMU interrupt number raised when code branches to EXC_RETURN
MAX_CHUNK = 10000              # max instructions per emu_start call
IDLE_REPEATS = 24             # identical MMIO polls before a busy-wait is fast-forwarded

FP_REGS = [A.UC_ARM_REG_S0 + i for i in range(16)] + [A.UC_ARM_REG_FPSCR]
KEEP_REGS = ([A.UC_ARM_REG_R0 + i for i in range(13)] +
             [A.UC_ARM_REG_LR, A.UC_ARM_REG_MSP, A.UC_ARM_REG_PSP, A.UC_ARM_REG_CONTROL,
              A.UC_ARM_REG_APSR, A.UC_ARM_REG_PRIMASK, A.UC_ARM_REG_BASEPRI, A.UC_ARM_REG_FAULTMASK,
              A.UC_ARM_REG_FPSCR] + [A.UC_ARM_REG_S0 + i for i in range(32)])
ENGINE_CALLS_LIMIT = 500_000  # rebuild the Unicorn engine after this many emu_start calls
FRAME_REGS = [A.UC_ARM_REG_R0, A.UC_ARM_REG_R1, A.UC_ARM_REG_R2, A.UC_ARM_REG_R3,
              A.UC_ARM_REG_R12, A.UC_ARM_REG_LR]

DEFAULT_HANDLER_NAMES = ("Default_Handler", "defaultISR", "IntDefaultHandler", "DefaultISR")
EXIT_NAMES = ("exit", "abort", "C$$EXIT")


class Machine:
    """One MSP432 LaunchPad.  Create, optionally attach a robot, then call run_until()."""

    def __init__(self, program, cpi=2.0, hle=True, idle_skip=True, log=None):
        self.prog = program
        self.cpi = cpi
        self.hle_enabled = hle
        self.idle_skip = idle_skip
        self.log = log or (lambda msg: None)
        self.messages = []                 # (sim time, text) shown in the UI / printed headless
        self._warned = set()
        self.unmodelled = {}
        self.trace_io = False
        self.halted = None                 # reason string once the CPU stops for good
        self.finished = False              # main() returned
        self.debug_halt = False            # stopped at a debugger breakpoint
        self.debug_skip_bp = None
        self.resets = 0
        self.uart_listeners = []           # fn(byte, t) for bytes sent on EUSCI_A0
        self.pwm_listeners = []            # fn(t) when any Timer_A output configuration changes
        self.analog_sources = {}           # (port, bit) -> fn(t) -> volts
        self.reset_listeners = []          # fn(t) after an MCU reset
        self.instructions = 0
        self.fpu_used = self._scan_fpu()

        self.sched = Scheduler()
        self.t = 0.0
        self.clk = {"MCLK": 3e6, "HSMCLK": 3e6, "SMCLK": 3e6, "ACLK": 32768.0, "BCLK": 32768.0}
        self.spi = self.cpi / self.clk["MCLK"]      # simulated seconds per instruction
        self._in_chunk = False
        self._now_override = None
        self._ipm = 4.0                             # running estimate: instructions per MMIO access

        self._build_peripherals()
        self._setup_cpu()

    # ------------------------------------------------------------------ setup
    def _build_peripherals(self):
        self.nvic = NVIC(self)
        self.systick = SysTick(self)
        self.cs = CS(self, 0x40010400)
        self.pcm = PCM(self, 0x40010000)
        self.flctl = FLCTL(self, 0x40011000)
        self.wdt = WDT(self, 0x40004800)
        self.ref = REF(self, 0x40003000)
        self.rstctl = RSTCTL(self, 0xE0042000)
        self.sysctl = Peripheral(self, 0xE0043000, "SYSCTL")      # SRAM bank / reset config: plain storage
        self.gpio = GPIO(self, 0x40004C00)
        self.timers = [TimerA(self, 0x40000000 + 0x400 * i, i) for i in range(4)]
        self.t32 = Timer32(self, 0x4000C000)
        self.adc = ADC14(self, 0x40012000)
        self.uarts = [EUSCI(self, 0x40001000 + 0x400 * i, f"EUSCI_A{i}", 16 + i) for i in range(4)]
        self.uarts += [EUSCI(self, 0x40002000 + 0x400 * i, f"EUSCI_B{i}", 20 + i, is_b=True) for i in range(4)]
        self.uart0 = self.uarts[0]
        self.uart0.tx_sink = self._uart_tx
        self.ppb = PPB(self, PPB_BASE, self.rstctl, self.sysctl)

        self.peripherals = [self.cs, self.pcm, self.flctl, self.wdt, self.ref, self.gpio,
                            *self.timers, self.t32, self.adc, *self.uarts]
        self.pages = [None] * (PERIPH_SIZE >> 10)
        for p in self.peripherals:
            off = p.base - PERIPH_BASE
            p.base_off = off
            for page in range(off >> 10, (off + max(p.size, 0x400) - 1 >> 10) + 1):
                self.pages[page] = p
        names = {0x3400: "COMP_E0", 0x3800: "COMP_E1", 0x3C00: "AES256", 0x4000: "CRC32",
                 0x4400: "RTC_C", 0x5000: "PMAP", 0x5400: "CAPTIO0", 0x5800: "CAPTIO1",
                 0xE000: "DMA", 0xF000: "DMA", 0x10800: "PSS"}
        for page in range(len(self.pages)):
            if self.pages[page] is None:
                off = page << 10
                u = Unmodelled(self, PERIPH_BASE + off, 0x400, names.get(off, f"periph@{PERIPH_BASE + off:08X}"))
                u.base_off = off
                self.pages[page] = u

        # interrupt sources (level sensitive)
        nv = self.nvic
        for t in self.timers:
            nv.add_source(t.irq0, t.irq0_line)
            nv.add_source(t.irqn, t.irqn_line)
        for u in self.uarts:
            nv.add_source(u.irq, u.irq_line)
        nv.add_source(ADC_IRQ, self.adc.irq_line)
        nv.add_source(T32_IRQ[0], self.t32.ch[0].line)
        nv.add_source(T32_IRQ[1], self.t32.ch[1].line)
        nv.add_source(T32_INTC, lambda: self.t32.ch[0].line() or self.t32.ch[1].line())
        for port in INTERRUPT_PORTS:
            nv.add_source(PORT_IRQ[port], lambda p=port: self.gpio.irq_line(p))

        self._pin_to_timer = {}
        for ti, chans in TIMER_PINS.items():
            for ch, pin in chans.items():
                self._pin_to_timer.setdefault(pin, []).append((ti, ch))

    def _scan_fpu(self):
        for addr, data in self.prog.segments:
            for i in range(0, len(data) - 3, 2):
                hw1 = data[i] | data[i + 1] << 8
                if hw1 >> 8 in (0xEC, 0xED, 0xEE):
                    hw2 = data[i + 2] | data[i + 3] << 8
                    if (hw2 >> 9) & 7 == 5:          # coprocessor 10/11 = VFP
                        return True
        return False

    def _setup_cpu(self):
        """Power-on / reset: fresh RAM, fresh CPU engine, registers from the vector table."""
        self.sram = ctypes.create_string_buffer(SRAM_SIZE)
        self.breakpoints = getattr(self, "breakpoints", set())
        self._create_engine()
        uc = self.uc
        sp, pc = struct.unpack("<II", bytes(uc.mem_read(0, 8)))
        uc.reg_write(A.UC_ARM_REG_SP, sp)
        self.pc = pc & ~1
        self._init_run_state()

    def _rebuild_engine(self):
        """Replace the Unicorn engine, keeping the CPU state.

        Works around a Unicorn 2.1 bug: with exit addresses enabled an engine crashes
        after ~2.4 million emu_start calls, so a fresh one is swapped in well before."""
        old = self.uc
        saved = {r: old.reg_read(r) for r in KEEP_REGS}
        self._create_engine()
        uc = self.uc
        uc.reg_write(A.UC_ARM_REG_CONTROL, saved.pop(A.UC_ARM_REG_CONTROL))
        for r, v in saved.items():
            uc.reg_write(r, v)

    def _create_engine(self):
        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
        self.uc = uc
        self._emu_calls = 0
        uc.mem_map(FLASH_BASE, FLASH_SIZE, UC_PROT_ALL)
        uc.mem_map(INFO_BASE, INFO_SIZE, UC_PROT_ALL)
        uc.mem_map_ptr(SRAM_BASE, SRAM_SIZE, UC_PROT_ALL, ctypes.addressof(self.sram))
        uc.mem_map_ptr(SRAM_CODE_ALIAS, SRAM_SIZE, UC_PROT_ALL, ctypes.addressof(self.sram))
        uc.mem_map(TRAMP, 0x1000, UC_PROT_ALL)
        uc.mem_write(TRAMP, b"\x70\x47" + b"\xfe\xe7" * 0x7FF)  # bx lr ; b . ...
        loaded = False
        for addr, data in self.prog.segments:
            in_flash = FLASH_BASE <= addr < FLASH_BASE + FLASH_SIZE
            in_ram = SRAM_BASE <= addr < SRAM_BASE + SRAM_SIZE
            if not (in_flash or in_ram or INFO_BASE <= addr < INFO_BASE + INFO_SIZE):
                raise ValueError(f"program segment at 0x{addr:08X} is outside MSP432 flash/RAM")
            uc.mem_write(addr, bytes(data))
            loaded = loaded or in_flash
        if not loaded:
            raise ValueError("program has nothing in flash - is this an MSP432 .out file?")
        uc.mem_protect(FLASH_BASE, FLASH_SIZE, UC_PROT_READ | UC_PROT_EXEC)
        uc.mem_protect(TRAMP, 0x1000, UC_PROT_READ | UC_PROT_EXEC)

        uc.mmio_map(PERIPH_BASE, PERIPH_SIZE, self._periph_read, None, self._periph_write, None)
        uc.mmio_map(BB_PERIPH_BASE, BB_PERIPH_SIZE, self._bbp_read, None, self._bbp_write, None)
        uc.mmio_map(BB_SRAM_BASE, BB_SRAM_SIZE, self._bbs_read, None, self._bbs_write, None)
        uc.mmio_map(PPB_BASE, PPB_SIZE, self._ppb_read, None, self._ppb_write, None)

        # Addresses where emulation must stop before executing (Unicorn "exits": unlike code
        # hooks they cost nothing per instruction).  Value = what to do there.
        self._exits = {}
        for addr in self.prog.find_halfwords(0xBF30):            # WFI instructions
            self._exits[addr] = ("wfi", None)
        if self.hle_enabled:
            for name in ("Clock_Delay1ms", "Clock_Delay1us", "delay"):
                a = self.prog.symbols.get(name)
                if a is not None:
                    self._exits[a] = ("hle", name)
        for name in EXIT_NAMES:
            a = self.prog.symbols.get(name)
            if a is not None:
                self._exits[a] = ("exit", name)
        # instructions that unmask interrupts: emulated here so pending interrupts are taken at once
        for a, (kind, reg) in self.prog.find_unmask_sites().items():
            self._exits.setdefault(a, ("unmask", (kind, reg)))
        uc.ctl_exits_enabled(True)
        self._apply_exits()
        uc.hook_add(UC_HOOK_MEM_UNMAPPED | UC_HOOK_MEM_PROT, self._on_bad_mem)
        uc.hook_add(UC_HOOK_INTR, self._on_intr)

        self.default_handler = None
        for n in DEFAULT_HANDLER_NAMES:
            if n in self.prog.symbols:
                self.default_handler = self.prog.symbols[n] & ~1
                break

        self._rb_entry = RegBatch(uc, FRAME_REGS + [A.UC_ARM_REG_XPSR, A.UC_ARM_REG_SP])
        self._rb_ret = RegBatch(uc, FRAME_REGS + [A.UC_ARM_REG_SP, A.UC_ARM_REG_APSR])
        self._rb_splr = RegBatch(uc, [A.UC_ARM_REG_SP, A.UC_ARM_REG_LR])
        self._rb_fp = RegBatch(uc, FP_REGS)
        self._rb_mask = RegBatch(uc, [A.UC_ARM_REG_PRIMASK, A.UC_ARM_REG_BASEPRI])

    def _init_run_state(self):
        self.sleeping = False
        self._wfi_spin = 0
        self._wfi_advance = False
        self._last_wfi_instr = -1
        self.stall = {}                     # exception depth -> remaining busy-delay seconds
        self.exc_fp = []                    # saved FP registers per active exception
        self._stop_reason = None
        self._want_stop = None
        self._replays = {}                  # pc -> [access records to replay on re-execution]
        self._reset_pending = False
        self._mem_fault = None
        self._idle_seen = {}
        self._idle_count = 0
        self._irq_dirty = True
        self.wdt.start()

    # ------------------------------------------------------------- utilities
    def now(self):
        """Current simulated time, also valid inside MMIO callbacks."""
        if self._now_override is not None:
            return self._now_override
        if self._in_chunk:
            k = min(self._mmio_count * self._ipm, self._chunk_n)
            return self._chunk_t0 + k * self.spi
        return self.t

    def pc_now(self):
        try:
            return self.uc.reg_read(A.UC_ARM_REG_PC)
        except Exception:
            return self.pc

    def where(self):
        return self.describe_pc(self.pc_now())

    def describe_pc(self, pc):
        if TRAMP <= pc < TRAMP + 0x1000:
            return "a Clock_Delay function (fast-forwarded)"
        return self.prog.describe(pc & ~1)

    def message(self, text):
        self.messages.append((self.now(), text))
        self.log(f"[{self.now():9.4f}s] {text}")

    def warn_once(self, key, text):
        if key in self._warned:
            return
        self._warned.add(key)
        self.message("warning: " + text)

    def note_unmodelled(self, addr, kind):
        key = (addr, kind)
        if key in self.unmodelled:
            return
        where = self.where()
        self.unmodelled[key] = where
        self.message(f"{kind} of {reg_name(addr)} at {where}: this register is not modelled "
                     f"(value stored, no effect)")

    def mark_dynamic(self):
        self._dyn = True

    def irq_maybe_changed(self):
        self._irq_dirty = True

    def pwm_changed(self, t):
        for fn in self.pwm_listeners:
            fn(t)

    def analog_input(self, channel, t):
        pin = CHANNEL_PINS.get(channel)
        fn = self.analog_sources.get(pin)
        return fn(t) if fn else 0.0

    def pin_edge(self, port, bit, rising, t):
        """External edge on a pin that may feed a Timer_A capture input."""
        for ti, ch in self._pin_to_timer.get((port, bit), ()):
            b = 1 << bit
            if self.gpio.sel0[port] & b and not self.gpio.sel1[port] & b:
                self.timers[ti].input_edge(ch, rising, t)

    def pin_drive(self, port, bit):
        """Average level (0..1) driven on a pin, or None if the pin is an input."""
        g = self.gpio
        b = 1 << bit
        if not g.dir[port] & b:
            return None
        if g.sel0[port] & b and not g.sel1[port] & b:
            for ti, ch in self._pin_to_timer.get((port, bit), ()):
                d = self.timers[ti].duty(ch)
                if d is not None:
                    return d
        return 1.0 if g.out[port] & b else 0.0

    def update_clocks(self):
        new = self.cs.frequencies()
        if new == self.clk:
            return
        t = self.now()
        old = dict(self.clk)
        self.clk.update(new)
        self.spi = self.cpi / self.clk["MCLK"]
        self.systick.clock_changed(t, old)
        self.ppb.clock_changed(t, old)
        for p in self.peripherals:
            p.clock_changed(t, old)
        if old["MCLK"] != new["MCLK"]:
            self.message(f"clock: MCLK {old['MCLK'] / 1e6:g} MHz -> {new['MCLK'] / 1e6:g} MHz, "
                         f"SMCLK {new['SMCLK'] / 1e6:g} MHz")
        if self._in_chunk:
            self._defer_stop("clock")

    def request_reset(self, reason):
        self._reset_reason = reason
        self._reset_pending = True
        if self._in_chunk:
            self._defer_stop("reset")

    def _uart_tx(self, byte, t):
        for fn in self.uart_listeners:
            fn(byte, t)

    def read32(self, addr):
        return struct.unpack("<I", bytes(self.uc.mem_read(addr, 4)))[0]

    def _stop(self, reason, data=None):
        """Stop emulation now.  Only safe from code hooks (before an instruction runs)."""
        if self._stop_reason in ("fault", "reset", "exit"):
            return
        self._stop_reason = reason
        self._stop_data = data
        self.uc.emu_stop()

    def _defer_stop(self, reason, data=None):
        """Request a stop from inside a register access; applied when the access completes."""
        if self._want_stop is None or reason in ("reset", "fault"):
            self._want_stop = (reason, data)

    # ---------------------------------------------------------------- MMIO
    # Unicorn reports the PC of the load/store itself when emulation is stopped from
    # an MMIO callback, so that instruction runs again on resume.  The access that
    # caused the stop is remembered and "replayed" (no side effects) the second time.
    def _replayed(self, kind, addr, size, value=None):
        """If this access is the re-execution of one that stopped emulation, return its record."""
        pc = self.uc.reg_read(A.UC_ARM_REG_PC)
        lst = self._replays.get(pc)
        if not lst:
            return None
        rp = lst[-1]
        if rp[0] == kind and rp[1] == addr and rp[2] == size and (kind == "R" or value is None or rp[3] == value):
            lst.pop()
            if not lst:
                del self._replays[pc]
            return rp
        return None

    def _finish_access(self, kind, addr, size, value):
        if self._want_stop is not None:
            reason, data = self._want_stop
            self._want_stop = None
            pc = self.uc.reg_read(A.UC_ARM_REG_PC)
            self._replays.setdefault(pc, []).append((kind, addr, size, value))
            self._stop(reason, data)

    def _idle_track(self, key, v):
        if not (self.idle_skip and self._in_chunk):
            return
        if self._dyn:
            self._idle_count = 0
            self._idle_seen.clear()
            return
        seen = self._idle_seen
        prev = seen.get(key)
        if prev == v:
            self._idle_count += 1
            if self._idle_count >= IDLE_REPEATS:
                self._defer_stop("idle")
        else:
            if prev is not None or len(seen) >= 8:
                seen.clear()
            seen[key] = v
            self._idle_count = 0

    def _periph_read(self, uc, off, size, ud):
        addr = PERIPH_BASE + off
        if self._replays:
            rp = self._replayed("R", addr, size)
            if rp:
                return rp[3]
        self._mmio_count += 1
        p = self.pages[off >> 10]
        self._dyn = False
        v = p.read(off - p.base_off, size)
        if self.trace_io:
            self.log(f"  R{size} {reg_name(addr):<22} -> 0x{v:0{2 * size}X}   {self.where()}")
        self._idle_track(off, v)
        self._finish_access("R", addr, size, v)
        return v

    def _periph_write(self, uc, off, size, value, ud):
        addr = PERIPH_BASE + off
        if self._replays and self._replayed("W", addr, size, value):
            return
        self._mmio_count += 1
        p = self.pages[off >> 10]
        if self.trace_io:
            self.log(f"  W{size} {reg_name(addr):<22} <- 0x{value:0{2 * size}X}   {self.where()}")
        p.write(off - p.base_off, size, value)
        self._idle_count = 0
        self._idle_seen.clear()
        if self._irq_dirty:
            self._check_irq_in_chunk()
        self._finish_access("W", addr, size, value)

    def _bbp_read(self, uc, off, size, ud):
        byte_off, bit = off >> 5, (off >> 2) & 7
        return (self._periph_read(uc, byte_off, 1, None) >> bit) & 1

    def _bbp_write(self, uc, off, size, value, ud):
        byte_off, bit = off >> 5, (off >> 2) & 7
        if self._replays and self._replayed("W", PERIPH_BASE + byte_off, 1, None) is not None:
            return                           # replay of a bit-band write that caused a stop
        p = self.pages[byte_off >> 10]
        cur = p.read(byte_off - p.base_off, 1)
        new = (cur | (1 << bit)) if value & 1 else (cur & ~(1 << bit))
        self._periph_write(uc, byte_off, 1, new, None)

    def _bbs_read(self, uc, off, size, ud):
        byte_off, bit = off >> 5, (off >> 2) & 7
        return (self.sram.raw[byte_off] >> bit) & 1

    def _bbs_write(self, uc, off, size, value, ud):
        byte_off, bit = off >> 5, (off >> 2) & 7
        cur = self.sram.raw[byte_off]
        new = (cur | (1 << bit)) if value & 1 else (cur & ~(1 << bit))
        ctypes.memmove(ctypes.addressof(self.sram) + byte_off, bytes([new]), 1)

    def _ppb_read(self, uc, off, size, ud):
        addr = PPB_BASE + off
        if self._replays:
            rp = self._replayed("R", addr, size)
            if rp:
                return rp[3]
        self._mmio_count += 1
        self._dyn = False
        v = self.ppb.read(off, size)
        if self.trace_io:
            self.log(f"  R{size} {reg_name(addr):<22} -> 0x{v:0{2 * size}X}   {self.where()}")
        self._idle_track(0x100000 + off, v)
        self._finish_access("R", addr, size, v)
        return v

    def _ppb_write(self, uc, off, size, value, ud):
        addr = PPB_BASE + off
        if self._replays and self._replayed("W", addr, size, value):
            return
        self._mmio_count += 1
        if self.trace_io:
            self.log(f"  W{size} {reg_name(addr):<22} <- 0x{value:0{2 * size}X}   {self.where()}")
        self.ppb.write(off, size, value)
        self._idle_count = 0
        self._idle_seen.clear()
        if self._irq_dirty:
            self._check_irq_in_chunk()
        self._finish_access("W", addr, size, value)

    def _check_irq_in_chunk(self):
        self._irq_dirty = False
        if not self._in_chunk:
            return
        primask, basepri = self._rb_mask.read()
        if self.nvic.best_pending(self.nvic.exec_priority(primask, basepri)) is not None:
            self._defer_stop("irq")

    # ---------------------------------------------------------- exits / hooks
    def _apply_exits(self, skip=None):
        addrs = [a for a in self._exits if a != skip] + [a for a in self.breakpoints if a != skip]
        self.uc.ctl_set_exits(addrs + [STOP_ADDR])

    def add_breakpoint(self, addr):
        self.breakpoints.add(addr & ~1)
        self._apply_exits()

    def remove_breakpoint(self, addr):
        self.breakpoints.discard(addr & ~1)
        self._apply_exits()

    def _handle_exit(self, pc):
        """Emulation stopped at one of our exit addresses (before executing it)."""
        if pc in self.breakpoints:
            self.debug_halt = True
            return
        kind, name = self._exits[pc]
        if kind == "wfi":
            self.pc = pc + 2
            self.sleeping = True
            if self.instructions - self._last_wfi_instr > 200:
                self._wfi_spin = 0               # not a tight WFI loop
            self._last_wfi_instr = self.instructions
        elif kind == "hle":
            n = self.uc.reg_read(A.UC_ARM_REG_R0)
            if name == "Clock_Delay1ms":
                dur = n * 1e-3
            elif name == "Clock_Delay1us":
                dur = n * 1e-6
            else:                               # delay(n): Valvano's calibrated loop, 9.162 cycles/iteration
                dur = n * 9.162 / self.clk["MCLK"]
            # resume at a 'bx lr' stub so the return works both for normal calls and for
            # tail calls made from an interrupt handler (LR = EXC_RETURN)
            self.pc = TRAMP
            if dur > 0:
                depth = len(self.nvic.active)
                self.stall[depth] = self.stall.get(depth, 0.0) + dur
        elif kind == "unmask":
            op, reg = name
            uc = self.uc
            if op == "cpsie":
                uc.reg_write(A.UC_ARM_REG_PRIMASK, 0)
                self.pc = pc + 2
            else:
                v = uc.reg_read(A.UC_ARM_REG_R0 + reg) if reg < 13 else 0
                if op == "primask":
                    uc.reg_write(A.UC_ARM_REG_PRIMASK, v & 1)
                elif op == "basepri":
                    uc.reg_write(A.UC_ARM_REG_BASEPRI, v & 0xFF)
                else:
                    cur = uc.reg_read(A.UC_ARM_REG_BASEPRI)
                    if v & 0xFF and (cur == 0 or (v & 0xFF) < cur):
                        uc.reg_write(A.UC_ARM_REG_BASEPRI, v & 0xFF)
                self.pc = pc + 4
            self.instructions += 1
        elif kind == "exit":
            # On the real chip the CPU now spins in exit(); peripherals (UART, motor PWM) keep going.
            self.sleeping = True
            if not self.finished:
                self.finished = True
                self.message("main() returned - the CPU now idles in exit(); peripherals keep running "
                             "(motors keep their last setting)")

    def _on_bad_mem(self, uc, access, addr, size, value, ud):
        self._mem_fault = (access, addr, size)
        return False

    def _on_intr(self, uc, intno, ud):
        if intno == EXCP_EXCEPTION_EXIT:
            self._exc_return(uc)
            return
        self._intr = intno
        names = {1: "undefined instruction", 2: "SVC instruction (not supported)",
                 3: "instruction fetch fault", 4: "data access fault", 7: "breakpoint instruction (BKPT)"}
        self._stop("fault", names.get(intno, f"CPU exception {intno}"))

    def _exc_return(self, uc):
        """An interrupt handler returned (branched to EXC_RETURN): pop the exception frame."""
        if not self.nvic.active:
            self._stop("fault", "jumped to an exception-return address outside of an interrupt")
            return
        sp = uc.reg_read(A.UC_ARM_REG_SP)
        try:
            r0, r1, r2, r3, r12, lr, pc, xpsr = struct.unpack("<8I", bytes(uc.mem_read(sp, 32)))
        except UcError:
            self._stop("fault", "stack pointer invalid on interrupt return")
            return
        self._rb_ret.write((r0, r1, r2, r3, r12, lr, sp + 32 + (4 if xpsr & 0x200 else 0), xpsr & 0xF80F0000))
        fp = self.exc_fp.pop()
        if fp:
            self._rb_fp.write(fp)
        self.nvic.active.pop()
        uc.reg_write(A.UC_ARM_REG_PC, pc | 1)
        # returning into code that is inside a skipped delay: let the main loop finish the delay
        if self.stall.get(len(self.nvic.active), 0.0) > 0:
            self._stop("stall")
            return
        # tail-chain / late arrival: stop so the main loop takes the next interrupt now
        nv = self.nvic
        if nv.latched or nv.enabled:
            primask, basepri = self._rb_mask.read()
            if nv.best_pending(nv.exec_priority(primask, basepri)) is not None:
                self._stop("irq")

    # ----------------------------------------------------------- exceptions
    def _enter_exception(self, exc):
        uc = self.uc
        r0, r1, r2, r3, r12, lr, xpsr, sp = self._rb_entry.read()
        vals = (r0, r1, r2, r3, r12, lr)
        align = sp & 4
        if align:
            sp -= 4
        sp -= 32
        stacked_psr = (xpsr & ~0x200 & ~0x1FF) | (0x200 if align else 0) | (1 << 24)
        frame = struct.pack("<8I", *vals, self.pc, stacked_psr)
        try:
            uc.mem_write(sp, frame)
        except UcError:
            self._fault(f"stack overflow: SP=0x{sp:08X} is outside RAM while entering {exception_name(exc)}")
            return
        self.exc_fp.append(self._rb_fp.read() if self.fpu_used else None)
        self.nvic.active.append(exc)
        self.nvic.clear_pending(exc)
        vtor = self.ppb.vtor
        try:
            handler = self.read32(vtor + 4 * exc)
        except UcError:
            self._fault(f"vector table at 0x{vtor:08X} is not readable")
            return
        if self.default_handler is not None and handler & ~1 == self.default_handler:
            self._fault(f"{exception_name(exc)} interrupt fired, but the program has no handler for it, "
                        f"so it would be stuck in Default_Handler forever. Either define the handler "
                        f"or don't enable this interrupt.")
            return
        self._rb_splr.write((sp, EXC_RETURN))
        self.pc = handler & ~1

    def _take_interrupt(self):
        nv = self.nvic
        if not nv.latched and not nv.enabled:
            return False
        primask, basepri = self._rb_mask.read()
        exc = nv.best_pending(nv.exec_priority(primask, basepri))
        if exc is None:
            if self.sleeping and not self.finished and nv.best_pending(nv.exec_priority(0, 0)) is not None:
                # WFI does not sleep while a masked interrupt is pending.  In a tight loop
                # (e.g. "DisableInterrupts(); while(1){ WaitForInterrupt(); }") let time move on
                # to the next event between passes instead of spinning millions of times.
                self._wfi_spin += 1
                if self._wfi_spin > 16 and not self._wfi_advance:
                    self._wfi_advance = True
                    return False                  # stay asleep until the next event
                self._wfi_advance = False
                self.sleeping = False
            return False
        self.sleeping = False
        self._enter_exception(exc)
        return True

    def _fault(self, text):
        self.halted = text
        self.message("STOPPED: " + text)

    # ------------------------------------------------------------- reset
    def _do_reset(self):
        reason = self._reset_reason
        self.resets += 1
        self.message(f"MCU reset: {reason}")
        self.sched.cancel_owner("mcu")
        for p in self.peripherals:
            p.reset()
        self.nvic.reset()
        self.systick.reset()
        self.ppb.reset()
        self.clk = {"MCLK": 3e6, "HSMCLK": 3e6, "SMCLK": 3e6, "ACLK": 32768.0, "BCLK": 32768.0}
        self.spi = self.cpi / self.clk["MCLK"]
        self._setup_cpu()
        for port in range(1, 13):
            self.gpio.ext_changed(port, self.t)
        for fn in self.reset_listeners:
            fn(self.t)

    # ------------------------------------------------------------ run loop
    def _fire_due(self):
        while True:
            ev = self.sched.pop_due(self.t)
            if ev is None:
                return
            self._now_override = ev.t
            try:
                ev.fn(ev.t)
            finally:
                self._now_override = None

    def run_until(self, t_end, wall_budget=None):
        """Run until simulated time t_end (or until halted / out of wall-clock budget)."""
        wall_deadline = time.perf_counter() + wall_budget if wall_budget else None
        iters = 0
        while self.t < t_end and not self.halted and not self.debug_halt:
            iters += 1
            if wall_deadline and iters & 63 == 0 and time.perf_counter() > wall_deadline:
                return
            if self._reset_pending:
                self._stop_reason = None
                self._do_reset()
            if self._emu_calls > ENGINE_CALLS_LIMIT:
                self._rebuild_engine()
            self._fire_due()
            if self._take_interrupt() and self.halted:
                return
            depth = len(self.nvic.active)
            stall = self.stall.get(depth, 0.0)
            if self.sleeping or stall > 0:
                target = min(self.sched.next_time(), t_end)
                if stall > 0:
                    target = min(target, self.t + stall)
                if target == math.inf:
                    target = t_end
                dt = max(0.0, target - self.t)
                self.t = max(self.t, target)
                if stall > 0:
                    left = stall - dt
                    if left <= 1e-12:
                        self.stall.pop(depth, None)
                    else:
                        self.stall[depth] = left
                continue
            if self.debug_skip_bp is not None:
                # resuming from a breakpoint: execute that one instruction without stopping on it
                skip, self.debug_skip_bp = self.debug_skip_bp, None
                if skip == self.pc:
                    self._apply_exits(skip=skip)
                    self._run_chunk(1, t_end)
                    self._apply_exits()
                    continue
            nxt = min(self.sched.next_time(), t_end)
            n = math.ceil((nxt - self.t) / self.spi) if nxt != math.inf else MAX_CHUNK
            n = max(1, min(MAX_CHUNK, n))
            self._run_chunk(n, t_end)

    def _run_chunk(self, n, t_end):
        uc = self.uc
        self._in_chunk = True
        self._chunk_t0 = self.t
        self._chunk_n = n
        self._mmio_count = 0
        self._stop_reason = None
        self._want_stop = None
        self._mem_fault = None
        err = None
        try:
            self._emu_calls += 1
            uc.emu_start(self.pc | 1, STOP_ADDR, count=n)    # 'until' is unused: exits are enabled
        except UcError as e:
            err = e
        finally:
            self._in_chunk = False
        reason = self._stop_reason
        pc = uc.reg_read(A.UC_ARM_REG_PC) & ~1
        at_exit = reason is None and err is None and (pc in self._exits or pc in self.breakpoints)
        if reason is None and err is None and not at_exit:
            executed = n
            if self._mmio_count:
                self._ipm = 0.9 * self._ipm + 0.1 * (n / self._mmio_count)
        else:
            executed = max(1, min(n, int(self._mmio_count * self._ipm)))
        self.instructions += executed
        self.t = self._chunk_t0 + executed * self.spi
        self.pc = pc & ~1
        if err is not None and reason != "reset":
            self._fault(self._describe_error(err, pc))
            return
        if at_exit:
            self._handle_exit(pc)
        elif reason == "idle":
            # busy-wait on a flag that only changes at an event: jump to the next event
            target = min(self.sched.next_time(), t_end)
            if target == math.inf:
                target = t_end
            self.t = max(self.t, target)
            self._idle_count = 0
            self._idle_seen.clear()
        elif reason == "fault":
            self._fault(f"{self._stop_data} at {self.prog.describe(self.pc)}")
        if reason not in ("reset",):
            self._stop_reason = None

    def _describe_error(self, err, pc):
        where = self.prog.describe(pc & ~1)
        mf = self._mem_fault
        if mf:
            access, addr, size = mf
            what = {UC_MEM_READ_UNMAPPED: "read from invalid address", UC_MEM_WRITE_UNMAPPED: "write to invalid address",
                    UC_MEM_FETCH_UNMAPPED: "jump to invalid address", UC_MEM_WRITE_PROT: "write to read-only (flash) address",
                    UC_MEM_FETCH_PROT: "execute non-executable address"}.get(access, "bad access to address")
            hint = ""
            if addr < 0x100:
                hint = " (looks like a NULL pointer)"
            elif 0x02000000 <= addr < 0x02100000:
                hint = " (TI driverlib ROM functions are not available in the simulator)"
            elif access == UC_MEM_WRITE_PROT and addr < FLASH_SIZE:
                hint = " (writing to flash or to a const variable)"
            elif SRAM_BASE - 0x1000 <= addr < SRAM_BASE:
                hint = " (stack overflow?)"
            return f"HardFault: {what} 0x{addr:08X}{hint} at {where}"
        return f"HardFault: {err} at {where}"

    def _advance_to(self, target):
        """Move time forward to `target`, firing every event on the way (CPU not running)."""
        while True:
            nt = self.sched.next_time()
            if nt > target:
                break
            self.t = max(self.t, nt)
            self._fire_due()
        self.t = max(self.t, target)
        self._fire_due()

    def step_instruction(self):
        """Debugger single step: execute one instruction (interrupts are masked while stepping)."""
        if self.halted:
            return
        depth = len(self.nvic.active)
        stall = self.stall.get(depth, 0.0)
        if stall > 0:                          # stepping over a skipped Clock_Delay: finish it
            self._advance_to(self.t + stall)
            self.stall.pop(depth, None)
            return
        if self.sleeping and not self.finished:  # stepping over WFI: wait for a wake-up event
            deadline = self.t + 10.0
            while self.sleeping and self.t < deadline:
                self._advance_to(min(self.sched.next_time(), deadline))
                if self.nvic.best_pending(self.nvic.exec_priority(0, 0)) is not None:
                    self.sleeping = False
            return
        self._fire_due()
        self._apply_exits(skip=self.pc)      # never stop on the instruction being stepped
        self._run_chunk(1, self.t + 1.0)
        self._apply_exits()
        self.debug_halt = False

    # ----------------------------------------------------------- user input
    def uart_input(self, data):
        if isinstance(data, str):
            data = data.encode("latin-1", "replace")
        self.uart0.feed_rx(data)

    def state_summary(self):
        return {
            "time": self.t,
            "pc": self.prog.describe(self.pc),
            "mclk": self.clk["MCLK"],
            "instructions": self.instructions,
            "halted": self.halted,
        }
