"""ADC14: single / sequence / repeat conversions of simulated analog inputs."""
from .base import Peripheral

IRQ = 24
SHT_CLOCKS = [4, 8, 16, 32, 64, 96, 128, 192]
RES_BITS = [8, 10, 12, 14]
RES_CLOCKS = [9, 11, 14, 16]
AVCC = 3.3

# ADC channel -> (port, bit) of the pin it samples on the MSP432P401R
CHANNEL_PINS = {
    0: (5, 5), 1: (5, 4), 2: (5, 3), 3: (5, 2), 4: (5, 1), 5: (5, 0), 6: (4, 7), 7: (4, 6),
    8: (4, 5), 9: (4, 4), 10: (4, 3), 11: (4, 2), 12: (4, 1), 13: (4, 0), 14: (6, 1), 15: (6, 0),
    16: (9, 1), 17: (9, 0), 18: (8, 7), 19: (8, 6), 20: (8, 5), 21: (8, 4), 22: (8, 3), 23: (8, 2),
}


class ADC14(Peripheral):
    name = "ADC14"
    size = 0x400

    def reset(self):
        self.ctl0 = 0
        self.ctl1 = 0x30
        self.mctl = [0] * 32
        self.memv = [0] * 32
        self.ier0 = 0
        self.ier1 = 0
        self.ifg0 = 0
        self.ifg1 = 0
        self.busy = False
        self.ev = None
        self.seq_pos = None

    # -- conversion engine ---------------------------------------------------
    def _adc_clock(self):
        ssel = (self.ctl0 >> 19) & 7
        clk = {0: 25e6, 1: 5e6, 2: self.m.clk["ACLK"], 3: self.m.clk["MCLK"],
               4: self.m.clk["SMCLK"], 5: self.m.clk["HSMCLK"]}.get(ssel, 25e6)
        pdiv = (1, 4, 32, 64)[(self.ctl0 >> 30) & 3]
        div = ((self.ctl0 >> 22) & 7) + 1
        return clk / pdiv / div

    def _conv_time(self, idx):
        sht_field = 8 if idx <= 7 or 24 <= idx else 12        # SHT0 for MEM0-7/24-31, SHT1 otherwise
        sht = SHT_CLOCKS[(self.ctl0 >> sht_field) & 7] if self.ctl0 & (1 << 26) else 4
        return (sht + RES_CLOCKS[(self.ctl1 >> 4) & 3]) / self._adc_clock()

    def _start(self, t):
        if self.busy or not self.ctl0 & 0x10:
            if not self.ctl0 & 0x10:
                self.m.warn_once("adc-off", "ADC conversion started while ADC14ON=0 - nothing happens")
            return
        self.busy = True
        self.seq_pos = (self.ctl1 >> 16) & 0x1F
        self.ev = self.m.sched.at(t + self._conv_time(self.seq_pos), self._done)

    def _sample(self, idx, t):
        mctl = self.mctl[idx]
        ch = mctl & 0x1F
        if ch == 22 and self.ctl1 & (1 << 23):
            volts = 0.7                                   # internal temperature sensor
        elif ch == 23 and self.ctl1 & (1 << 22):
            volts = AVCC / 2                              # battery monitor (AVCC/2)
        else:
            volts = self.m.analog_input(ch, t)
        vrsel = (mctl >> 8) & 0xF
        vref = self.m.ref.vref() if vrsel in (1, 15) else AVCC
        bits = RES_BITS[(self.ctl1 >> 4) & 3]
        full = (1 << bits) - 1
        code = int(round(max(0.0, min(volts, vref)) / vref * full))
        if self.ctl1 & 0x08:
            self.m.warn_once("adc-df", "ADC signed (DF=1) result format is not modelled; results are unsigned")
        return code

    def _done(self, t):
        self.ev = None
        idx = self.seq_pos
        self.memv[idx] = self._sample(idx, t)
        if idx < 32:
            self.ifg0 |= 1 << idx
        conseq = (self.ctl0 >> 17) & 3
        last = conseq in (0, 2) or (self.mctl[idx] & 0x80) or idx == 31
        if not last:
            self.seq_pos = idx + 1
            self.ev = self.m.sched.at(t + self._conv_time(self.seq_pos), self._done)
        else:
            self.busy = False
            self.ctl0 &= ~1                               # SC clears when conversion completes
            if conseq in (2, 3) and self.ctl0 & 2:        # repeat modes keep converting while ENC=1
                self.busy = True
                self.seq_pos = (self.ctl1 >> 16) & 0x1F
                self.ev = self.m.sched.at(t + self._conv_time(self.seq_pos), self._done)
        self.m.irq_maybe_changed()

    def irq_line(self):
        return bool(self.ier0 & self.ifg0)

    # -- registers -----------------------------------------------------------
    def read(self, off, size):
        reg = off & ~3
        if reg == 0x00:
            v = self.ctl0 | (0x10000 if self.busy else 0)
        elif reg == 0x04:
            v = self.ctl1
        elif 0x18 <= reg < 0x98:
            v = self.mctl[(reg - 0x18) >> 2]
        elif 0x98 <= reg < 0x118:
            i = (reg - 0x98) >> 2
            v = self.memv[i]
            self.ifg0 &= ~(1 << i)
        elif reg == 0x13C:
            v = self.ier0
        elif reg == 0x140:
            v = self.ier1
        elif reg == 0x144:
            v = self.ifg0
        elif reg == 0x148:
            v = self.ifg1
        elif reg == 0x154:
            pend = self.ifg0 & self.ier0
            if pend:
                i = (pend & -pend).bit_length() - 1
                self.ifg0 &= ~(1 << i)
                v = 0x0C + 2 * i
            else:
                v = 0
        else:
            self.m.note_unmodelled(self.base + off, "read")
            v = self.raw(reg, 4)
        return (v >> (8 * (off & 3))) & ((1 << 8 * size) - 1)

    def write(self, off, size, value):
        reg = off & ~3
        t = self.m.now()
        if size < 4:
            cur = self.read(reg, 4) if not 0x98 <= reg < 0x118 else 0
            shift = 8 * (off & 3)
            mask = ((1 << 8 * size) - 1) << shift
            value = (cur & ~mask) | ((value << shift) & mask)
        value &= 0xFFFFFFFF
        if reg == 0x00:
            old = self.ctl0
            self.ctl0 = value & ~0x10000
            if not value & 2 and self.busy and (old >> 17) & 3 in (2, 3):
                pass                                       # repeat sequence stops after current conversion
            if value & 1 and value & 2:
                self._start(t)
        elif reg == 0x04:
            self.ctl1 = value
        elif 0x18 <= reg < 0x98:
            self.mctl[(reg - 0x18) >> 2] = value
        elif reg == 0x13C:
            self.ier0 = value
        elif reg == 0x140:
            self.ier1 = value
        elif reg == 0x14C:
            self.ifg0 &= ~value
        elif reg == 0x150:
            self.ifg1 &= ~value
        else:
            self.m.note_unmodelled(self.base + off, "write")
            self.set_raw(reg, 4, value)
        self.m.irq_maybe_changed()
