"""Fast batched register access for Unicorn (bypasses the per-call Python wrapper cost)."""
import ctypes

try:
    from unicorn.unicorn_py3.unicorn import uclib as _uclib
except Exception:              # pragma: no cover - other unicorn layouts
    _uclib = None


class RegBatch:
    """Read or write a fixed list of 32-bit registers with one C call."""

    def __init__(self, uc, regs):
        self.uc = uc
        self.regs = list(regs)
        n = len(self.regs)
        self.n = n
        self.ids = (ctypes.c_int * n)(*self.regs)
        self.vals = (ctypes.c_uint32 * n)()
        base = ctypes.addressof(self.vals)
        self.ptrs = (ctypes.c_void_p * n)(*[base + 4 * i for i in range(n)])
        self.handle = getattr(uc, "_uch", None)
        self.fast = _uclib is not None and self.handle is not None

    def read(self):
        if self.fast:
            _uclib.uc_reg_read_batch(self.handle, self.ids, self.ptrs, self.n)
            return self.vals[:]
        return [self.uc.reg_read(r) for r in self.regs]

    def write(self, values):
        if self.fast:
            self.vals[:] = [v & 0xFFFFFFFF for v in values]
            _uclib.uc_reg_write_batch(self.handle, self.ids, self.ptrs, self.n)
            return
        for r, v in zip(self.regs, values):
            self.uc.reg_write(r, v)
