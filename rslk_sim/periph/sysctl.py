"""Clock system (CS), power control (PCM), flash controller, watchdog, REF_A, reset control."""
from .base import Peripheral

HFXT_HZ = 48_000_000      # LaunchPad high-frequency crystal
LFXT_HZ = 32_768
VLO_HZ = 9_400
MODOSC_HZ = 25_000_000
DCO_RANGES = [1_500_000, 3_000_000, 6_000_000, 12_000_000, 24_000_000, 48_000_000]


class CS(Peripheral):
    name = "CS"
    KEY, CTL0, CTL1, CTL2, CTL3, CLKEN, STAT = 0x00, 0x04, 0x08, 0x0C, 0x10, 0x30, 0x34
    IE, IFG, CLRIFG, SETIFG = 0x40, 0x48, 0x50, 0x58

    def reset(self):
        super().reset()
        self.locked = True
        self.set_raw(self.CTL0, 4, 0x00010000)   # DCORSEL = 1 -> 3 MHz
        self.set_raw(self.CTL1, 4, 0x00000033)   # MCLK = SMCLK = DCO, ACLK = LFXT(->REFO)
        self.set_raw(self.CTL2, 4, 0x00010003)
        self.set_raw(self.CTL3, 4, 0x000000BB)
        self.set_raw(self.CLKEN, 4, 0x0000000F)

    def read(self, off, size):
        if off >> 2 == self.KEY >> 2:
            return (0xA596 if self.locked else 0x0000) >> (8 * (off & 3)) & ((1 << 8 * size) - 1)
        if off >> 2 == self.STAT >> 2:
            stat = 0x1FFF00F7  # every oscillator on, every clock ready
            return stat >> (8 * (off & 3)) & ((1 << 8 * size) - 1)
        if off >> 2 == self.IFG >> 2:
            return 0           # no oscillator faults
        return self.raw(off, size)

    def write(self, off, size, value):
        reg = off & ~3
        if reg == self.KEY:
            _, new = self.merge(reg, 4, off, size, value)
            self.locked = (new & 0xFFFF) != 0x695A
            return
        if reg in (self.STAT, self.IFG, self.CLRIFG, self.SETIFG):
            return
        if self.locked:
            self.m.warn_once(f"cs-locked-{reg}", f"write to CS register 0x{self.base + reg:08X} while CS is locked (write 0x695A to CS->KEY first) - ignored")
            return
        _, new = self.merge(reg, 4, off, size, value)
        self.set_raw(reg, 4, new)
        if reg in (self.CTL0, self.CTL1, self.CLKEN):
            self.m.update_clocks()

    def frequencies(self):
        ctl0 = self.raw(self.CTL0, 4)
        ctl1 = self.raw(self.CTL1, 4)
        clken = self.raw(self.CLKEN, 4)
        dcorsel = (ctl0 >> 16) & 7
        dco = DCO_RANGES[min(dcorsel, 5)]
        refo = 128_000 if clken & (1 << 15) else 32_768
        src = {0: LFXT_HZ, 1: VLO_HZ, 2: refo, 3: dco, 4: MODOSC_HZ, 5: HFXT_HZ, 6: HFXT_HZ, 7: HFXT_HZ}
        selm, sels, sela = ctl1 & 7, (ctl1 >> 4) & 7, (ctl1 >> 8) & 7
        divm, divhs, diva, divs = (ctl1 >> 16) & 7, (ctl1 >> 20) & 7, (ctl1 >> 24) & 7, (ctl1 >> 28) & 7
        return {
            "MCLK": src[selm] / (1 << divm),
            "HSMCLK": src[sels] / (1 << divhs),
            "SMCLK": src[sels] / (1 << divs),
            "ACLK": src[sela if sela <= 2 else 0] / (1 << diva),
            "BCLK": refo if ctl1 & (1 << 12) else LFXT_HZ,
        }


class PCM(Peripheral):
    name = "PCM"

    def reset(self):
        super().reset()
        self.cpm = 0

    def read(self, off, size):
        reg = off & ~3
        if reg == 0x00:
            v = 0xA5960000 | (self.cpm << 8) | (self.raw(0, 4) & 0xFF)
        elif reg == 0x04:
            v = self.raw(4, 4) & ~0x100        # PMR_BUSY always clear
        elif reg == 0x0C:
            v = 0                               # no invalid-transition flags
        else:
            v = self.raw(reg, 4)
        return v >> (8 * (off & 3)) & ((1 << 8 * size) - 1)

    def write(self, off, size, value):
        reg = off & ~3
        _, new = self.merge(reg, 4, off, size, value)
        if reg in (0x00, 0x04):
            if (new >> 16) != 0x695A:
                self.m.warn_once(f"pcm-key-{reg}", "PCM write without the 0x695A key - ignored")
                return
            if reg == 0x00:
                self.cpm = new & 0xF            # power mode transition completes instantly
            self.set_raw(reg, 4, new & 0xFFFF)
            return
        self.set_raw(reg, 4, new)


class FLCTL(Peripheral):
    """Flash controller: wait-state configuration is accepted silently; programming is not modelled."""
    name = "FLCTL"
    size = 0x1000

    def write(self, off, size, value):
        reg = off & ~3
        if reg not in (0x10, 0x14):            # BANK0/1_RDCTL hold wait states (fine to store)
            self.m.note_unmodelled(self.base + off, "write")
        super().write(off, size, value)

    def read(self, off, size):
        reg = off & ~3
        if reg not in (0x10, 0x14):
            self.m.note_unmodelled(self.base + off, "read")
        return super().read(off, size)


class WDT(Peripheral):
    """Watchdog timer: resets the program if it is left running and never fed."""
    name = "WDT_A"
    INTERVALS = [1 << 31, 1 << 27, 1 << 23, 1 << 19, 1 << 15, 1 << 13, 1 << 9, 1 << 6]
    IRQ = 3

    def reset(self):
        super().reset()
        self.ctl = 0x04          # SMCLK, 2^15 cycles, running
        self.ev = None
        self.irq_flag = False

    def start(self):
        self._restart(self.m.now())

    def read(self, off, size):
        if off & ~1 == 0x0C:
            v = 0x6900 | self.ctl
            return v >> (8 * (off & 1)) & ((1 << 8 * size) - 1)
        return 0

    def write(self, off, size, value):
        if off & ~1 != 0x0C:
            return
        if size == 1 or (value >> 8) & 0xFF != 0x5A:
            self.m.request_reset("watchdog password violation (WDT_A->CTL must be written as 0x5A00 | bits)")
            return
        self.ctl = value & 0xF7          # CNTCL reads back as 0
        self._restart(self.m.now())

    def _restart(self, t):
        if self.ev:
            self.ev.cancel()
            self.ev = None
        if self.ctl & 0x80:              # WDTHOLD
            return
        ssel = (self.ctl >> 5) & 3
        clk = self.m.clk["SMCLK"] if ssel == 0 else self.m.clk["ACLK"] if ssel == 1 else VLO_HZ if ssel == 2 else self.m.clk["BCLK"]
        period = self.INTERVALS[self.ctl & 7] / clk
        self.ev = self.m.sched.at(t + period, self._expire)

    def _expire(self, t):
        self.ev = None
        if self.ctl & 0x10:              # interval-timer mode: periodic interrupt
            self.irq_flag = True
            self.m.nvic.set_pending(16 + self.IRQ)
            self._restart(t)
        else:
            self.m.request_reset("watchdog timeout (the watchdog was left running and not fed; "
                                 "stop it with WDT_A->CTL = 0x5A80)")

    def clock_changed(self, t, old):
        pass


class REF(Peripheral):
    name = "REF_A"

    def read(self, off, size):
        if off & ~1 == 0x00:
            v = self.raw(0, 2) & 0x08FB
            if v & 1:                       # REFON: generator and bandgap ready, never busy
                v |= 0x3300
            return v >> (8 * (off & 1)) & ((1 << 8 * size) - 1)
        return self.raw(off, size)

    def vref(self):
        v = self.raw(0, 2)
        return {0: 1.2, 1: 1.45, 2: 2.5, 3: 2.5}[(v >> 4) & 3]


class RSTCTL(Peripheral):
    name = "RSTCTL"

    def write(self, off, size, value):
        if off & ~3 == 0x00 and ((value >> 8) & 0xFF) == 0x69 and value & 3:
            self.m.request_reset("software reset requested through RSTCTL")
            return
        super().write(off, size, value)
