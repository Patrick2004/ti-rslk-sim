"""Timer32 module (two 32/16-bit down counters clocked from MCLK)."""
import math

from .base import Peripheral

T32_IRQ = (25, 26)
T32_INTC = 27


class T32Channel:
    def __init__(self, m, idx):
        self.m = m
        self.idx = idx
        self.reset()

    def reset(self):
        self.load = 0
        self.bgload = 0
        self.control = 0x20
        self.ris = 0
        self.t0 = 0.0
        self.v0 = 0xFFFFFFFF
        self.f = 0.0
        self.ev = None
        self.stopped = False

    def _mask(self):
        return 0xFFFFFFFF if self.control & 2 else 0xFFFF

    def _freq(self):
        pre = (self.control >> 2) & 3
        return self.m.clk["MCLK"] / (1, 16, 256, 256)[pre]

    def _enabled(self):
        return bool(self.control & 0x80) and not self.stopped

    def _reload(self):
        return self.load & self._mask() if self.control & 0x40 else self._mask()

    def value(self, t):
        v0 = self.v0 & self._mask()
        if not self._enabled() or self.f == 0:
            return v0
        k = math.floor((t - self.t0) * self.f + 1e-6)
        if k <= v0:
            return v0 - k
        if self.control & 1:        # one-shot: stays at zero
            return 0
        reload = self._reload()
        return reload - ((k - v0 - 1) % (reload + 1))

    def _anchor(self, t, v):
        self.t0, self.v0, self.f = t, v & self._mask(), self._freq()

    def _next_zero(self, t):
        if not self._enabled() or self.f == 0:
            return math.inf
        v0 = self.v0 & self._mask()
        k_now = math.floor((t - self.t0) * self.f + 1e-6)
        if k_now < v0:
            k = v0
        else:
            if self.control & 1:
                return math.inf
            period = self._reload() + 1
            k = v0 + ((k_now - v0) // period + 1) * period
        return self.t0 + k / self.f

    def schedule(self, t):
        if self.ev:
            self.ev.cancel()
            self.ev = None
        if self._enabled() and (not self.ris or self.control & 0x20):
            tn = self._next_zero(t)
            if tn != math.inf:
                self.ev = self.m.sched.at(tn, self._zero)

    def _zero(self, t):
        self.ev = None
        self.ris = 1
        if self.control & 1:
            self.stopped = True
            self._anchor(t, 0)
        self.m.irq_maybe_changed()
        self.schedule(t + 1e-12)

    def line(self):
        return bool(self.ris and self.control & 0x20)

    def read(self, reg):
        t = self.m.now()
        if reg == 0x00:
            return self.load
        if reg == 0x04:
            self.m.mark_dynamic()
            return self.value(t)
        if reg == 0x08:
            return self.control
        if reg == 0x10:
            return self.ris
        if reg == 0x14:
            return self.ris if self.control & 0x20 else 0
        if reg == 0x18:
            return self.bgload
        return 0

    def write(self, reg, value):
        t = self.m.now()
        if reg == 0x00:
            self.load = self.bgload = value
            self.stopped = False
            self._anchor(t, value)
        elif reg == 0x08:
            cur = self.value(t)
            was = self.control & 0x80
            self.control = value & 0xEF
            if not was and value & 0x80:
                self.stopped = False
            self._anchor(t, cur)
        elif reg == 0x0C:
            self.ris = 0
        elif reg == 0x18:
            self.bgload = value
            self.load = value      # takes effect at the next reload
        self.schedule(t)
        self.m.irq_maybe_changed()

    def clock_changed(self, t):
        v = self.value(t)
        self._anchor(t, v)
        self.schedule(t)


class Timer32(Peripheral):
    name = "TIMER32"
    size = 0x400

    def __init__(self, m, base):
        self.ch = [T32Channel(m, 0), T32Channel(m, 1)]
        super().__init__(m, base)

    def reset(self):
        for c in self.ch:
            c.reset()

    def read(self, off, size):
        if off >= 0x40:
            return 0
        c = self.ch[off // 0x20]
        v = c.read(off % 0x20 & ~3)
        return (v >> (8 * (off & 3))) & ((1 << 8 * size) - 1)

    def write(self, off, size, value):
        if off >= 0x40:
            return
        c = self.ch[off // 0x20]
        c.write(off % 0x20 & ~3, value & 0xFFFFFFFF)

    def clock_changed(self, t, old):
        for c in self.ch:
            c.clock_changed(t)
