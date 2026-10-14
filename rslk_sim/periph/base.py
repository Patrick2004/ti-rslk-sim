"""Base class for memory-mapped peripheral models."""


class Peripheral:
    """A block of registers.  Default behaviour is plain storage (byte array).

    Subclasses override read()/write() for registers with side effects and
    call the raw helpers for the rest.  Offsets are relative to `base`.
    """
    name = "?"
    size = 0x400

    def __init__(self, m, base, name=None):
        self.m = m
        self.base = base
        if name:
            self.name = name
        self.mem = bytearray(self.size)
        self.reset()

    # -- raw storage helpers -------------------------------------------------
    def raw(self, off, size):
        return int.from_bytes(self.mem[off:off + size], "little")

    def set_raw(self, off, size, value):
        self.mem[off:off + size] = (value & ((1 << (8 * size)) - 1)).to_bytes(size, "little")

    def merge(self, reg_off, reg_size, off, size, value):
        """Apply a (possibly partial) write of `size` bytes at `off` to the register
        at reg_off/reg_size.  Returns (old, new) register values."""
        old = self.raw(reg_off, reg_size)
        shift = 8 * (off - reg_off)
        mask = ((1 << (8 * size)) - 1) << shift
        new = (old & ~mask) | ((value << shift) & mask)
        new &= (1 << (8 * reg_size)) - 1
        return old, new

    # -- interface -----------------------------------------------------------
    def reset(self):
        self.mem[:] = bytes(self.size)

    def read(self, off, size):
        return self.raw(off, size)

    def write(self, off, size, value):
        self.set_raw(off, size, value)

    def clock_changed(self, t, old):
        """Called after any system clock frequency changes; `old` is the previous clock dict."""


class Unmodelled(Peripheral):
    """Storage-only block that logs the first access to each register."""

    def __init__(self, m, base, size, name):
        self.size = size
        super().__init__(m, base, name)

    def read(self, off, size):
        self.m.note_unmodelled(self.base + off, "read")
        return self.raw(off, size)

    def write(self, off, size, value):
        self.m.note_unmodelled(self.base + off, "write")
        self.set_raw(off, size, value)
