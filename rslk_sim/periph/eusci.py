"""eUSCI_A (UART / SPI) and eUSCI_B (SPI) serial modules."""
from collections import deque

from .base import Peripheral

RXIFG, TXIFG, STTIFG, TXCPTIFG = 0x1, 0x2, 0x4, 0x8
UCSWRST, UCSYNC = 0x1, 0x100


class EUSCI(Peripheral):
    size = 0x400

    def __init__(self, m, base, name, irq, is_b=False):
        self.is_b = is_b
        self.irq = irq
        self.name = name
        self.off_stat = 0x08 if is_b else 0x0A
        self.off_ie = 0x2A if is_b else 0x1A
        self.off_ifg = 0x2C if is_b else 0x1C
        self.off_iv = 0x2E if is_b else 0x1E
        self.rx_queue = deque()
        self.spi_device = None             # object with .spi_byte(byte, t) -> response byte
        self.tx_sink = None                # fn(byte, t) for UART output
        super().__init__(m, base, name)

    def reset(self):
        super().reset()
        self.ctlw0 = 0x01C1 if self.is_b else 0x0001
        self.brw = 0
        self.mctlw = 0
        self.ie = 0
        self.ifg = TXIFG
        self.rxbuf = 0
        self.tx_pending = None             # byte waiting in TXBUF while the shifter is busy
        self.shifting = False
        self.ev_tx = None
        self.ev_rx = None
        self.rx_ready_t = 0.0
        self.rx_queue.clear()

    # -- timing --------------------------------------------------------------
    def _clk(self):
        ssel = (self.ctlw0 >> 6) & 3
        if ssel == 1:
            return self.m.clk["ACLK"]
        if ssel == 0:
            self.m.warn_once(f"{self.name}-uclk", f"{self.name} clocked from external UCLK (not modelled); assuming SMCLK")
        return self.m.clk["SMCLK"]

    def _sync(self):
        return bool(self.ctlw0 & UCSYNC)

    def char_time(self):
        clk = self._clk()
        brw = self.brw or 1
        if self._sync():
            return 8 * brw / clk
        if self.mctlw & 1:                 # UCOS16 oversampling
            div = 16 * brw + ((self.mctlw >> 4) & 0xF)
        else:
            div = brw
        baud = clk / div
        c = self.ctlw0
        bits = 1 + (7 if c & 0x1000 else 8) + (1 if c & 0x8000 else 0) + (2 if c & 0x0800 else 1)
        return bits / baud

    def baud(self):
        return 10 / self.char_time() if not self._sync() else None

    # -- transmit ------------------------------------------------------------
    def _tx_write(self, byte, t):
        if self.ctlw0 & UCSWRST:
            return
        if self._sync() and (self.ctlw0 >> 9) & 3 == 3:
            self.m.warn_once(f"{self.name}-i2c", f"{self.name} I2C mode is not modelled")
            return
        self.ifg &= ~(TXIFG | TXCPTIFG)
        if not self.shifting:
            self._start_shift(byte, t)
        else:
            self.tx_pending = byte

    def _start_shift(self, byte, t):
        self.shifting = True
        self.ifg |= TXIFG                  # TXBUF empty again as soon as the shifter takes the byte
        self.ev_tx = self.m.sched.at(t + self.char_time(), lambda tt, b=byte: self._shift_done(b, tt))
        self.m.irq_maybe_changed()

    def _shift_done(self, byte, t):
        self.ev_tx = None
        self.shifting = False
        if self._sync():
            resp = 0xFF
            if self.spi_device is not None:
                r = self.spi_device.spi_byte(byte, t)
                resp = 0xFF if r is None else r & 0xFF
            self.rxbuf = resp
            self.ifg |= RXIFG
        else:
            if self.tx_sink:
                self.tx_sink(byte, t)
        if self.tx_pending is not None:
            b, self.tx_pending = self.tx_pending, None
            self._start_shift(b, t)
        else:
            self.ifg |= TXCPTIFG
        self.m.irq_maybe_changed()

    # -- receive (UART) ------------------------------------------------------
    def feed_rx(self, data):
        """Queue bytes typed by the user; delivered at the configured baud rate."""
        self.rx_queue.extend(data)
        self._schedule_rx(self.m.now())

    def _schedule_rx(self, t):
        if self.ev_rx or not self.rx_queue or self.ifg & RXIFG or self.ctlw0 & UCSWRST:
            return
        when = max(t, self.rx_ready_t)
        self.ev_rx = self.m.sched.at(when, self._rx_deliver)

    def _rx_deliver(self, t):
        self.ev_rx = None
        if self.ctlw0 & UCSWRST or self.ifg & RXIFG or not self.rx_queue:
            return
        self.rxbuf = self.rx_queue.popleft()
        self.ifg |= RXIFG
        self.rx_ready_t = t + self.char_time()
        self.m.irq_maybe_changed()

    def irq_line(self):
        return bool(self.ie & self.ifg)

    # -- registers -----------------------------------------------------------
    def read(self, off, size):
        reg = off & ~1
        t = self.m.now()
        if reg == 0x00:
            v = self.ctlw0
        elif reg == 0x06:
            v = self.brw
        elif reg == 0x08 and not self.is_b:
            v = self.mctlw
        elif reg == self.off_stat:
            v = 1 if (self.shifting or self.tx_pending is not None) else 0    # UCBUSY
        elif reg == 0x0C:
            v = self.rxbuf
            if self.ifg & RXIFG:
                self.ifg &= ~RXIFG
                self._schedule_rx(t)
        elif reg == 0x0E:
            v = 0
        elif reg == self.off_ie:
            v = self.ie
        elif reg == self.off_ifg:
            v = self.ifg
        elif reg == self.off_iv:
            pend = self.ie & self.ifg
            v = 0
            for bit, code in ((RXIFG, 2), (TXIFG, 4), (STTIFG, 6), (TXCPTIFG, 8)):
                if pend & bit:
                    self.ifg &= ~bit
                    v = code
                    break
        else:
            v = self.raw(reg, 2)
        return (v >> (8 * (off & 1))) & ((1 << 8 * size) - 1)

    def write(self, off, size, value):
        reg = off & ~1
        t = self.m.now()
        if size == 1 and reg not in (0x0E,):
            cur = self.read(reg, 2) if reg not in (0x0C, self.off_iv) else 0
            shift = 8 * (off & 1)
            value = (cur & ~(0xFF << shift)) | ((value & 0xFF) << shift)
        value &= 0xFFFF
        if reg == 0x00:
            was_reset = self.ctlw0 & UCSWRST
            self.ctlw0 = value
            if value & UCSWRST and not was_reset:
                self.ie = 0
                self.ifg = TXIFG
                self.tx_pending = None
            if was_reset and not value & UCSWRST:
                self.ifg |= TXIFG
                self._schedule_rx(t)
        elif reg == 0x06:
            self.brw = value
        elif reg == 0x08 and not self.is_b:
            self.mctlw = value
        elif reg == 0x0E:
            self._tx_write(value & 0xFF, t)
        elif reg == self.off_ie:
            self.ie = value & 0xF
        elif reg == self.off_ifg:
            self.ifg = value & 0xF
            self._schedule_rx(t)
        elif reg in (0x0C, self.off_iv, self.off_stat):
            pass
        else:
            self.set_raw(reg, 2, value)
        self.m.irq_maybe_changed()
