"""Timer_A0..A3: counting modes, compare flags, input capture, PWM outputs."""
import math

from .base import Peripheral

CTL, R, EX0, IV = 0x00, 0x10, 0x20, 0x2E
CCIFG, COV, OUT, CCI, CCIE, CAP = 0x01, 0x02, 0x04, 0x08, 0x10, 0x100
TAIFG, TAIE, TACLR = 0x01, 0x02, 0x04
NCH = 5

# Pins that carry TAx.n (capture input CCIxA and PWM output), MSP432P401R default mapping
TIMER_PINS = {
    0: {0: (7, 3), 1: (2, 4), 2: (2, 5), 3: (2, 6), 4: (2, 7)},
    1: {0: (8, 0), 1: (7, 7), 2: (7, 6), 3: (7, 5), 4: (7, 4)},
    2: {0: (8, 1), 1: (5, 6), 2: (5, 7), 3: (6, 6), 4: (6, 7)},
    3: {0: (10, 4), 1: (10, 5), 2: (8, 2), 3: (9, 2), 4: (9, 3)},
}


class TimerA(Peripheral):
    size = 0x400

    def __init__(self, m, base, index):
        self.index = index
        self.name = f"TIMER_A{index}"
        self.irq0 = 8 + 2 * index      # TAx_0
        self.irqn = 9 + 2 * index      # TAx_N
        super().__init__(m, base)

    def reset(self):
        self.ctl = 0
        self.cctl = [0] * NCH
        self.ccr = [0] * NCH
        self.ex0 = 0
        self.t0 = 0.0          # anchor time
        self.q0 = 0            # anchor position in the counting cycle
        self.f = 0.0           # tick frequency at anchor
        self.k_sync = 0        # ticks elapsed at last flag sync
        self.ev = None
        self.cci_level = [0] * NCH

    # -- counting model ------------------------------------------------------
    def _mode(self):
        return (self.ctl >> 4) & 3

    def _tick_freq(self):
        tassel = (self.ctl >> 8) & 3
        if tassel == 1:
            clk = self.m.clk["ACLK"]
        elif tassel == 2:
            clk = self.m.clk["SMCLK"]
        else:
            if self._mode() != 0:
                self.m.warn_once(f"ta{self.index}-extclk", f"{self.name} uses an external clock input (TASSEL={tassel}), which is not modelled - timer stopped")
            return 0.0
        return clk / (1 << ((self.ctl >> 6) & 3)) / ((self.ex0 & 7) + 1)

    def _cycle(self):
        """Length of the counting cycle in ticks, or 0 if the timer does not count."""
        mode = self._mode()
        top = self.ccr[0]
        if mode == 0 or self.f == 0:
            return 0
        if mode == 1:
            return top + 1 if top else 0
        if mode == 2:
            return 0x10000
        return 2 * top if top else 0

    def _ticks(self, t):
        if self.f == 0:
            return 0
        return math.floor((t - self.t0) * self.f + 1e-6)

    def _pos(self, t):
        L = self._cycle()
        if not L:
            return self.q0
        return (self.q0 + self._ticks(t)) % L

    def _value_from_pos(self, q):
        if self._mode() == 3 and self.ccr[0] and q > self.ccr[0]:
            return 2 * self.ccr[0] - q
        return q

    def value(self, t):
        return self._value_from_pos(self._pos(t)) & 0xFFFF

    def _anchor(self, t, value=None, down=False):
        """Restart the counting model at time t with the given counter value."""
        if value is None:
            q = self._pos(t)
        else:
            q = value
            if self._mode() == 3 and down and self.ccr[0]:
                q = 2 * self.ccr[0] - value
        self.f = self._tick_freq()
        L = self._cycle()
        if L and q >= L:
            q %= L
        self.t0, self.q0, self.k_sync = t, q, 0

    # -- flags ---------------------------------------------------------------
    def _flag_positions(self):
        """[(position, kind, channel)] for every flag that is currently clear."""
        mode = self._mode()
        L = self._cycle()
        if not L:
            return []
        top = self.ccr[0]
        out = []
        if not self.ctl & TAIFG:
            out.append((0, "taifg", None))
        for ch in range(NCH):
            c = self.cctl[ch]
            if c & CAP or c & CCIFG:
                continue
            x = self.ccr[ch]
            if mode == 1:
                if x <= top:
                    out.append((x, "cc", ch))
            elif mode == 2:
                out.append((x, "cc", ch))
            else:
                if x == 0 or x >= top:
                    out.append((min(x, top), "cc", ch))
                else:
                    out.append((x, "cc", ch))
                    out.append((2 * top - x, "cc", ch))
        return out

    def _sync(self, t):
        """Set every flag whose event happened in (k_sync, k_now]."""
        L = self._cycle()
        if not L:
            return
        k_now = self._ticks(t)
        k_last = self.k_sync
        if k_now <= k_last:
            return
        q_last = (self.q0 + k_last) % L
        span = k_now - k_last
        for x, kind, ch in self._flag_positions():
            dk = (x - q_last - 1) % L + 1          # ticks until the counter next reaches x
            if dk <= span:
                if kind == "taifg":
                    self.ctl |= TAIFG
                else:
                    self.cctl[ch] |= CCIFG
        self.k_sync = k_now

    def _schedule(self, t):
        if self.ev:
            self.ev.cancel()
            self.ev = None
        L = self._cycle()
        if not L:
            return
        k_now = self._ticks(t)
        q_now = (self.q0 + k_now) % L
        best = None
        for x, _, _ in self._flag_positions():
            dk = (x - q_now - 1) % L + 1
            if best is None or dk < best:
                best = dk
        if best is not None:
            self.ev = self.m.sched.at(self.t0 + (k_now + best) / self.f, self._on_event)

    def _on_event(self, t):
        self.ev = None
        self._sync(t)
        self._schedule(t)
        self.m.irq_maybe_changed()

    def irq0_line(self):
        c = self.cctl[0]
        return bool(c & CCIE and c & CCIFG)

    def irqn_line(self):
        if self.ctl & TAIE and self.ctl & TAIFG:
            return True
        for ch in range(1, NCH):
            c = self.cctl[ch]
            if c & CCIE and c & CCIFG:
                return True
        return False

    # -- capture -------------------------------------------------------------
    def input_edge(self, ch, rising, t):
        """An edge arrived on capture input CCIxA of channel ch."""
        self.cci_level[ch] = 1 if rising else 0
        c = self.cctl[ch]
        if not c & CAP or (c >> 12) & 3 != 0:
            return
        self._capture(ch, rising, t)

    def _capture(self, ch, rising, t):
        c = self.cctl[ch]
        cm = (c >> 14) & 3
        if not (cm == 3 or (cm == 1 and rising) or (cm == 2 and not rising)):
            return
        self._sync(t)
        if c & CCIFG:
            c |= COV
        self.ccr[ch] = self.value(t)
        self.cctl[ch] = c | CCIFG
        self.m.irq_maybe_changed()

    # -- PWM output ----------------------------------------------------------
    def duty(self, ch):
        """Average output level (0..1) of TAx.ch, or None if it does not toggle."""
        c = self.cctl[ch]
        outmod = (c >> 5) & 7
        if c & CAP:
            return None
        if outmod == 0 or ch == 0:
            return 1.0 if c & OUT else 0.0
        mode = self._mode()
        top = self.ccr[0]
        if not self._cycle():
            return 1.0 if c & OUT else 0.0
        x = min(self.ccr[ch], top)
        # event list for one period: (tick, kind) with kind 'n' = reach CCRn, '0' = reach CCR0
        if mode == 1:
            # TAR runs 0..CCR0; the CCR0 event takes effect as the count rolls over to 0,
            # giving the documented duty = CCRn / (CCR0 + 1)
            period = top + 1
            events = [(period, "0")] if self.ccr[ch] > top else [(self.ccr[ch], "n"), (period, "0")]
        elif mode == 3:
            period = 2 * top
            events = [(x, "n"), (top, "0"), (2 * top - x, "n")]
        else:
            period = 0x10000
            events = [(x, "n"), (top, "0")]
        events.sort(key=lambda e: e[0])

        def apply(level, kind):
            if kind == "n":
                return {1: 1, 2: 1 - level, 3: 1, 4: 1 - level, 5: 0, 6: 1 - level, 7: 0}[outmod]
            return {1: level, 2: 0, 3: 0, 4: level, 5: level, 6: 1, 7: 1}[outmod]

        level = 0
        for _ in range(2):                    # first pass reaches steady state
            high = 0
            prev = 0
            for tick, kind in events:
                high += level * (tick - prev)
                level = apply(level, kind)
                prev = tick
            high += level * (period - prev)
        return high / period

    # -- register interface --------------------------------------------------
    def read(self, off, size):
        reg = off & ~1
        t = self.m.now()
        if reg == CTL:
            self._sync(t)
            v = self.ctl & ~TACLR
        elif 0x02 <= reg <= 0x0A:
            self._sync(t)
            ch = (reg - 0x02) >> 1
            v = (self.cctl[ch] & ~CCI) | (CCI if self.cci_level[ch] else 0)
        elif reg == R:
            self.m.mark_dynamic()
            v = self.value(t)
        elif 0x12 <= reg <= 0x1A:
            v = self.ccr[(reg - 0x12) >> 1]
        elif reg == EX0:
            v = self.ex0
        elif reg == IV:
            self._sync(t)
            v = self._read_iv(t)
        else:
            v = 0
        v = (v >> (8 * (off & 1))) & ((1 << 8 * size) - 1)
        return v

    def _read_iv(self, t):
        for ch in range(1, NCH):
            c = self.cctl[ch]
            if c & CCIE and c & CCIFG:
                self.cctl[ch] = c & ~CCIFG
                self._schedule(t)
                return 2 * ch
        if self.ctl & TAIE and self.ctl & TAIFG:
            self.ctl &= ~TAIFG
            self._schedule(t)
            return 0x0E
        return 0

    def write(self, off, size, value):
        reg = off & ~1
        t = self.m.now()
        self._sync(t)
        if size == 1:                          # byte write: merge into the 16-bit register
            cur = self.read(reg, 2) if reg != IV else 0
            shift = 8 * (off & 1)
            value = (cur & ~(0xFF << shift)) | ((value & 0xFF) << shift)
        value &= 0xFFFF
        if reg == CTL:
            cur_val = self.value(t)
            down = self._is_counting_down(t)
            self.ctl = value & ~TACLR
            if value & TACLR:
                self._anchor(t, 0)
            else:
                self._anchor(t, cur_val, down)
        elif 0x02 <= reg <= 0x0A:
            ch = (reg - 0x02) >> 1
            old = self.cctl[ch]
            self.cctl[ch] = value & ~CCI
            if old & CAP and value & CAP:
                a, b = (old >> 12) & 3, (value >> 12) & 3
                if a == 2 and b == 3:
                    self._capture(ch, True, t)       # software capture: GND -> VCC
                elif a == 3 and b == 2:
                    self._capture(ch, False, t)
        elif reg == R:
            self._anchor(t, value)
        elif 0x12 <= reg <= 0x1A:
            ch = (reg - 0x12) >> 1
            if ch == 0:
                cur_val = self.value(t)
                down = self._is_counting_down(t)
                self.ccr[0] = value
                if self._mode() == 1 and cur_val > value:
                    self._anchor(t, 0)               # counter rolls to zero immediately
                else:
                    self._anchor(t, min(cur_val, value) if self._mode() == 3 else cur_val, down)
            else:
                self.ccr[ch] = value
        elif reg == EX0:
            cur_val = self.value(t)
            self.ex0 = value & 7
            self._anchor(t, cur_val, self._is_counting_down(t))
        self._schedule(t)
        self.m.pwm_changed(t)
        self.m.irq_maybe_changed()

    def _is_counting_down(self, t):
        return self._mode() == 3 and self.ccr[0] and self._pos(t) > self.ccr[0]

    def clock_changed(self, t, old):
        self._sync(t)
        v = self.value(t)
        down = self._is_counting_down(t)
        self._anchor(t, v, down)
        self._schedule(t)
        self.m.pwm_changed(t)
