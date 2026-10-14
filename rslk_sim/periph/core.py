"""Cortex-M4 private peripheral bus: NVIC, SysTick, SCB, DWT (0xE0000000-0xE00FFFFF)."""
import math

from .base import Peripheral

EXC_SVCALL, EXC_PENDSV, EXC_SYSTICK = 11, 14, 15

IRQ_NAMES = [
    "PSS", "CS", "PCM", "WDT_A", "FPU", "FLCTL", "COMP_E0", "COMP_E1", "TA0_0", "TA0_N",
    "TA1_0", "TA1_N", "TA2_0", "TA2_N", "TA3_0", "TA3_N", "EUSCIA0", "EUSCIA1", "EUSCIA2",
    "EUSCIA3", "EUSCIB0", "EUSCIB1", "EUSCIB2", "EUSCIB3", "ADC14", "T32_INT1", "T32_INT2",
    "T32_INTC", "AES256", "RTC_C", "DMA_ERR", "DMA_INT3", "DMA_INT2", "DMA_INT1", "DMA_INT0",
    "PORT1", "PORT2", "PORT3", "PORT4", "PORT5", "PORT6",
]
SYS_NAMES = {2: "NMI", 3: "HardFault", 4: "MemManage", 5: "BusFault", 6: "UsageFault",
             11: "SVCall", 12: "DebugMon", 14: "PendSV", 15: "SysTick"}


def exception_name(exc):
    if exc >= 16:
        n = exc - 16
        return f"{IRQ_NAMES[n]}_IRQHandler (IRQ {n})" if n < len(IRQ_NAMES) else f"IRQ {n}"
    return f"{SYS_NAMES.get(exc, f'exception {exc}')}_Handler"


class NVIC:
    """Interrupt controller state plus the exception-priority logic."""

    def __init__(self, m):
        self.m = m
        self.sources = {}          # irq number -> callable() -> bool  (level-sensitive lines)
        self.reset()

    def reset(self):
        self.enabled = 0           # IRQ enable bits (ISER)
        self.latched = 0           # pending bits by exception number (software / pulse)
        self.prio = bytearray(64)  # IP[n]
        self.shpr = bytearray(12)  # system handler priority bytes, exceptions 4..15
        self.active = []           # stack of active exception numbers

    def add_source(self, irq, fn):
        self.sources[irq] = fn

    def set_pending(self, exc):
        self.latched |= 1 << exc
        self.m.irq_maybe_changed()

    def clear_pending(self, exc):
        self.latched &= ~(1 << exc)

    def priority(self, exc):
        if exc >= 16:
            return self.prio[exc - 16] & 0xE0       # MSP432 implements 3 priority bits
        if exc >= 4:
            return self.shpr[exc - 4] & 0xE0
        return -3 + exc - 1 if exc in (2, 3) else -3

    def is_pending(self, exc):
        if self.latched >> exc & 1:
            return True
        if exc >= 16:
            src = self.sources.get(exc - 16)
            return bool(src and src())
        return False

    def best_pending(self, threshold):
        """Highest-priority pending exception with priority < threshold, or None."""
        best, best_p = None, threshold
        lat = self.latched
        en = self.enabled
        # latched exceptions
        while lat:
            bit = lat & -lat
            exc = bit.bit_length() - 1
            lat ^= bit
            if exc >= 16 and not (en >> (exc - 16)) & 1:
                continue
            p = self.priority(exc)
            if p < best_p or (p == best_p and best is not None and exc < best):
                best, best_p = exc, p
        # level-sensitive peripheral lines of enabled IRQs
        e = en
        while e:
            bit = e & -e
            irq = bit.bit_length() - 1
            e ^= bit
            src = self.sources.get(irq)
            if src is None:
                continue
            exc = irq + 16
            if exc in self.active:
                continue
            p = self.prio[irq] & 0xE0
            if (p < best_p or (p == best_p and best is not None and exc < best)) and src():
                best, best_p = exc, p
        return best

    def exec_priority(self, primask, basepri):
        p = 256
        if self.active:
            p = min(self.priority(e) for e in self.active)
        if basepri & 0xE0:
            p = min(p, basepri & 0xE0)
        if primask:
            p = min(p, 0)
        return p

    # -- register access (offsets relative to 0xE000E000) -------------------------
    def read(self, off):
        """32-bit register read; off is word aligned."""
        if 0x100 <= off < 0x108:
            return (self.enabled >> (32 * ((off - 0x100) >> 2))) & 0xFFFFFFFF
        if 0x180 <= off < 0x188:
            return (self.enabled >> (32 * ((off - 0x180) >> 2))) & 0xFFFFFFFF
        if 0x200 <= off < 0x208 or 0x280 <= off < 0x288:
            word = ((off & 0x7F) >> 2)
            bits = 0
            for i in range(32):
                irq = 32 * word + i
                if self.is_pending(irq + 16):
                    bits |= 1 << i
            return bits
        if 0x300 <= off < 0x308:
            word = (off - 0x300) >> 2
            bits = 0
            for e in self.active:
                if e >= 16 and (e - 16) // 32 == word:
                    bits |= 1 << ((e - 16) % 32)
            return bits
        if 0x400 <= off < 0x440:
            i = off - 0x400
            return int.from_bytes(self.prio[i:i + 4], "little")
        return 0

    def write(self, off, value):
        if 0x100 <= off < 0x108:
            self.enabled |= (value & 0xFFFFFFFF) << (32 * ((off - 0x100) >> 2))
        elif 0x180 <= off < 0x188:
            self.enabled &= ~((value & 0xFFFFFFFF) << (32 * ((off - 0x180) >> 2)))
        elif 0x200 <= off < 0x208:
            self.latched |= (value & 0xFFFFFFFF) << (32 * ((off - 0x200) >> 2) + 16)
        elif 0x280 <= off < 0x288:
            self.latched &= ~((value & 0xFFFFFFFF) << (32 * ((off - 0x280) >> 2) + 16))
        elif 0x400 <= off < 0x440:
            i = off - 0x400
            self.prio[i:i + 4] = (value & 0xFFFFFFFF).to_bytes(4, "little")
        self.m.irq_maybe_changed()


class SysTick:
    def __init__(self, m):
        self.m = m
        self.reset()

    def reset(self):
        self.ctrl = 0
        self.load = 0
        self.t0 = 0.0
        self.v0 = 0
        self.countflag = False
        self.ev = None

    def _freq(self):
        return self.m.clk["MCLK"]

    def value(self, t):
        if not self.ctrl & 1:
            return self.v0
        k = math.floor((t - self.t0) * self._freq() + 1e-6)
        if k <= self.v0:
            return self.v0 - k
        if self.load == 0:
            return 0
        k2 = k - self.v0 - 1
        return self.load - (k2 % (self.load + 1))

    def _next_zero(self, t):
        """Time of the next 1->0 transition strictly after t (or inf)."""
        if not self.ctrl & 1:
            return math.inf
        f = self._freq()
        k_now = math.floor((t - self.t0) * f + 1e-6)
        period = self.load + 1
        first = self.v0 if self.v0 > 0 else self.v0 + period
        if self.load == 0 and self.v0 == 0:
            return math.inf
        if k_now < first:
            k = first
        else:
            if self.load == 0:
                return math.inf
            k = first + ((k_now - first) // period + 1) * period
        return self.t0 + k / f

    def _schedule(self, t):
        if self.ev:
            self.ev.cancel()
            self.ev = None
        if self.ctrl & 1 and (self.ctrl & 2 or not self.countflag):
            tn = self._next_zero(t)
            if tn != math.inf:
                self.ev = self.m.sched.at(tn, self._zero)

    def _zero(self, t):
        self.ev = None
        self.countflag = True
        if self.ctrl & 2:
            self.m.nvic.set_pending(EXC_SYSTICK)
        self._schedule(t + 1e-12)

    def read(self, off):
        t = self.m.now()
        if off == 0x10:
            v = self.ctrl | (0x10000 if self.countflag else 0)
            if self.countflag:
                self.countflag = False
                self._schedule(t)
            return v
        if off == 0x14:
            return self.load
        if off == 0x18:
            self.m.mark_dynamic()
            return self.value(t)
        if off == 0x1C:
            return 0x40000000 | 0      # NOREF, no calibration value
        return 0

    def write(self, off, value):
        t = self.m.now()
        if off == 0x10:
            was_on = self.ctrl & 1
            if was_on:
                self.v0, self.t0 = self.value(t), t
            self.ctrl = value & 0x7
            if not was_on and self.ctrl & 1:
                self.t0 = t
            if self.ctrl & 1 and not self.ctrl & 4:
                self.m.warn_once("systick-clksrc", "SysTick CLKSOURCE=0 selected; the simulator clocks SysTick from MCLK anyway")
        elif off == 0x14:
            self.load = value & 0xFFFFFF
        elif off == 0x18:
            self.v0, self.t0 = 0, t
            self.countflag = False
        self._schedule(t)

    def clock_changed(self, t, old):
        # re-anchor at the old frequency, then continue at the new one
        if self.ctrl & 1:
            f_new = self.m.clk["MCLK"]
            self.m.clk["MCLK"] = old["MCLK"]
            v = self.value(t)
            self.m.clk["MCLK"] = f_new
            self.v0, self.t0 = v, t
        self._schedule(t)


class PPB(Peripheral):
    """System control space and friends.  Unknown registers are stored and logged."""
    name = "PPB"
    size = 0           # sparse; storage is a dict

    def __init__(self, m, base, rstctl, sysctl):
        self.store = {}
        self.rstctl = rstctl
        self.sysctl = sysctl
        super().__init__(m, base)

    def reset(self):
        self.store = {}
        self.vtor = 0
        self.shpr_raw = bytearray(12)
        self.dwt_ctrl = 0
        self.cyc_base = 0.0
        self.cyc_t = 0.0
        self.scr = 0

    def cycles(self, t):
        return self.cyc_base + (t - self.cyc_t) * self.m.clk["MCLK"]

    def clock_changed(self, t, old):
        self.cyc_base += (t - self.cyc_t) * old["MCLK"]
        self.cyc_t = t

    def read(self, off, size):
        if 0x42000 <= off < 0x43000:
            return self.rstctl.read(off - 0x42000, size)
        if 0x43000 <= off < 0x45000:
            return self.sysctl.read(off - 0x43000, size)
        word = off & ~3
        v = self._read32(word)
        return (v >> (8 * (off & 3))) & ((1 << 8 * size) - 1)

    def _read32(self, off):
        m = self.m
        if 0xE010 <= off < 0xE020:
            return m.systick.read(off - 0xE000)
        if 0xE100 <= off < 0xE500:
            return m.nvic.read(off - 0xE000)
        if off == 0xED00:
            return 0x410FC241                     # Cortex-M4 r0p1
        if off == 0xED04:
            vect = m.nvic.active[-1] if m.nvic.active else 0
            pend = m.nvic.best_pending(256) or 0
            st = (1 << 26) if m.nvic.latched >> EXC_SYSTICK & 1 else 0
            return vect | (pend << 12) | st
        if off == 0xED08:
            return self.vtor
        if off == 0xED0C:
            return 0xFA050000
        if off == 0xED10:
            return self.scr
        if 0xED18 <= off < 0xED24:
            i = off - 0xED18
            return int.from_bytes(m.nvic.shpr[i:i + 4], "little")
        if off == 0x1000:
            return self.dwt_ctrl
        if off == 0x1004:
            m.mark_dynamic()
            return int(self.cycles(m.now())) & 0xFFFFFFFF if self.dwt_ctrl & 1 else self.store.get(off, 0)
        if off not in (0xED14, 0xED24, 0xED88, 0xEDFC, 0xEF34, 0xEF38):
            m.note_unmodelled(self.base + off, "read")
        return self.store.get(off, 0)

    def write(self, off, size, value):
        if 0x42000 <= off < 0x43000:
            return self.rstctl.write(off - 0x42000, size, value)
        if 0x43000 <= off < 0x45000:
            return self.sysctl.write(off - 0x43000, size, value)
        word = off & ~3
        if size < 4:
            # merge partial write into the current 32-bit value (byte writes to IP/SHPR are common)
            cur = self._peek32(word)
            shift = 8 * (off & 3)
            mask = ((1 << 8 * size) - 1) << shift
            value = (cur & ~mask) | ((value << shift) & mask)
        self._write32(word, value & 0xFFFFFFFF)

    def _peek32(self, off):
        m = self.m
        if 0xE400 <= off < 0xE440:
            i = off - 0xE400
            return int.from_bytes(m.nvic.prio[i:i + 4], "little")
        if 0xED18 <= off < 0xED24:
            i = off - 0xED18
            return int.from_bytes(m.nvic.shpr[i:i + 4], "little")
        if off == 0xE010:
            return m.systick.ctrl
        if off == 0xE014:
            return m.systick.load
        return self.store.get(off, 0)

    def _write32(self, off, value):
        m = self.m
        if 0xE010 <= off < 0xE020:
            m.systick.write(off - 0xE000, value)
        elif 0xE100 <= off < 0xE500:
            m.nvic.write(off - 0xE000, value)
        elif off == 0xED04:                         # ICSR
            if value & (1 << 26):
                m.nvic.set_pending(EXC_SYSTICK)
            if value & (1 << 25):
                m.nvic.clear_pending(EXC_SYSTICK)
            if value & (1 << 28):
                m.nvic.set_pending(EXC_PENDSV)
            if value & (1 << 27):
                m.nvic.clear_pending(EXC_PENDSV)
        elif off == 0xED08:
            self.vtor = value & 0xFFFFFF80
        elif off == 0xED0C:                         # AIRCR
            if value >> 16 == 0x05FA and value & 4:
                m.request_reset("NVIC_SystemReset (SYSRESETREQ)")
        elif off == 0xED10:
            self.scr = value
            if value & 2:
                m.warn_once("sleeponexit", "SCR.SLEEPONEXIT is not modelled; the CPU returns to thread mode normally")
        elif 0xED18 <= off < 0xED24:
            i = off - 0xED18
            m.nvic.shpr[i:i + 4] = value.to_bytes(4, "little")
        elif off == 0xEF00:                         # STIR
            m.nvic.set_pending(16 + (value & 0x3F))
        elif off == 0x1000:
            if value & 1 and not self.dwt_ctrl & 1:
                self.cyc_base = float(self.store.get(0x1004, 0))
                self.cyc_t = m.now()
            self.dwt_ctrl = value
        elif off == 0x1004:
            self.store[off] = value
            self.cyc_base, self.cyc_t = float(value), m.now()
        else:
            if off not in (0xED14, 0xED24, 0xED88, 0xEDFC, 0xEF34, 0xEF38):
                m.note_unmodelled(self.base + off, "write")
            self.store[off] = value
