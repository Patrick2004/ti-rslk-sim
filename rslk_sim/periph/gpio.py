"""Digital I/O ports P1-P10 and PJ (DIO block at 0x40004C00)."""
from .base import Peripheral

PJ = 11
NPORTS = 13          # indices 1..10 = P1..P10, 11 = PJ, 12 = unused high byte of PJ
INTERRUPT_PORTS = range(1, 7)
PORT_IRQ = {n: 34 + n for n in INTERRUPT_PORTS}   # PORT1_IRQn = 35 ... PORT6_IRQn = 40

# byte-register offsets inside a port pair (low byte = odd port, high byte = even port)
R_IN, R_OUT, R_DIR, R_REN, R_DS, R_SEL0, R_SEL1 = 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C
R_IV_ODD, R_SELC, R_IES, R_IE, R_IFG, R_IV_EVEN = 0x0E, 0x16, 0x18, 0x1A, 0x1C, 0x1E
REG_NAMES = {R_OUT: "out", R_DIR: "dir", R_REN: "ren", R_DS: "ds", R_SEL0: "sel0",
             R_SEL1: "sel1", R_IES: "ies", R_IE: "ie", R_IFG: "ifg"}
OUTPUT_REGS = (R_OUT, R_DIR, R_REN, R_SEL0, R_SEL1, R_SELC)


class GPIO(Peripheral):
    name = "DIO"
    size = 0x400

    def __init__(self, m, base):
        self.inputs = [[] for _ in range(NPORTS)]     # providers: fn(t) -> (mask, value)
        self.ext_mask = [0] * NPORTS
        self.ext_val = [0] * NPORTS
        self.listeners = [[] for _ in range(NPORTS)]  # fn(t) called when outputs change
        super().__init__(m, base)

    def reset(self):
        for r in ("out", "dir", "ren", "ds", "sel0", "sel1", "ies", "ie", "ifg", "level"):
            setattr(self, r, [0] * NPORTS)

    # -- device side ---------------------------------------------------------
    def attach_input(self, port, fn):
        """Register an external driver for some pins of `port`.  fn(t) -> (mask, value)."""
        self.inputs[port].append(fn)
        self.ext_changed(port, self.m.now())

    def on_output(self, port, fn):
        self.listeners[port].append(fn)

    def ext_changed(self, port, t):
        """An external device changed what it drives onto `port`."""
        mask = val = 0
        for fn in self.inputs[port]:
            mk, v = fn(t)
            val = (val & ~mk) | (v & mk)
            mask |= mk
        self.ext_mask[port], self.ext_val[port] = mask, val
        self._update_level(port)

    def pin_in(self, port):
        """Logic level seen on every pin of `port` (what PxIN returns)."""
        d, o = self.dir[port], self.out[port]
        mk, v = self.ext_mask[port], self.ext_val[port]
        undriven = ~d & ~mk & 0xFF
        return ((d & o) | (~d & mk & v) | (undriven & self.ren[port] & o)) & 0xFF

    def is_output_high(self, port, bit):
        b = 1 << bit
        return bool(self.dir[port] & self.out[port] & b)

    def irq_line(self, port):
        return bool(self.ifg[port] & self.ie[port])

    def _update_level(self, port):
        new = self.pin_in(port)
        if port in INTERRUPT_PORTS:
            old = self.level[port]
            changed = old ^ new
            if changed:
                rising, falling = changed & new, changed & old
                ies = self.ies[port]
                self.ifg[port] |= (rising & ~ies | falling & ies) & 0xFF
        self.level[port] = new

    # -- register interface --------------------------------------------------
    @staticmethod
    def _decode(off):
        """Return (odd_port_index, register_offset_in_pair) or (None, None)."""
        if off < 0xA0:
            return 2 * (off >> 5) + 1, off & 0x1F
        if 0x120 <= off < 0x140:
            return PJ, off - 0x120
        return None, None

    def _iv(self, port):
        pend = self.ifg[port] & self.ie[port]
        if not pend:
            return 0
        bit = (pend & -pend).bit_length() - 1
        self.ifg[port] &= ~(1 << bit)
        return 2 * (bit + 1)

    def read(self, off, size):
        first, r = self._decode(off)
        if first is None:
            return 0
        if r & ~1 == R_IV_ODD or r & ~1 == R_IV_EVEN:
            port = first if r & ~1 == R_IV_ODD else first + 1
            v = self._iv(port) if (r & 1) == 0 else 0
            return v & ((1 << 8 * size) - 1)
        val = 0
        for i in range(size):
            val |= self._read_byte(first, r + i) << (8 * i)
        return val

    def _read_byte(self, first, r):
        port = first + (r & 1)
        reg = r & ~1
        if reg == R_IN:
            return self.pin_in(port)
        name = REG_NAMES.get(reg)
        if name:
            return getattr(self, name)[port]
        if reg == R_SELC:
            return 0
        return 0

    def write(self, off, size, value):
        first, r = self._decode(off)
        if first is None:
            return
        t = self.m.now()
        touched = set()
        for i in range(size):
            rr = r + i
            if rr & ~1 in (R_IV_ODD, R_IV_EVEN):
                continue
            port = first + (rr & 1)
            reg = rr & ~1
            b = (value >> (8 * i)) & 0xFF
            if reg == R_SELC:
                self.sel0[port] ^= b
                self.sel1[port] ^= b
            else:
                name = REG_NAMES.get(reg)
                if not name:
                    continue
                getattr(self, name)[port] = b
            if reg in OUTPUT_REGS:
                touched.add(port)
        for port in touched:
            self._update_level(port)
            for fn in self.listeners[port]:
                fn(t)
        self.m.irq_maybe_changed()
