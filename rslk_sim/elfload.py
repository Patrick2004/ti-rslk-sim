"""ELF loading, symbols and (best effort) source-line lookup for CCS .out files."""
import bisect

from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection


class Program:
    def __init__(self, path):
        self.path = path
        self.segments = []          # (addr, bytes) for PT_LOAD segments with file data
        self.symbols = {}           # name -> address (Thumb bit cleared)
        self.code_ranges = []       # (start, end) of executable segments
        self._func_addrs = []       # sorted function start addresses
        self._func_names = []
        self._func_sizes = []
        self._lines = None          # sorted [(addr, file, line)] built lazily
        self._line_addrs = None
        with open(path, "rb") as f:
            self._elf_bytes = f.read()
        import io
        self._elf = ELFFile(io.BytesIO(self._elf_bytes))
        self._load()

    def _load(self):
        elf = self._elf
        if elf["e_machine"] != "EM_ARM":
            raise ValueError(f"{self.path} is not an ARM ELF file")
        for seg in elf.iter_segments():
            if seg["p_type"] != "PT_LOAD":
                continue
            data = seg.data()
            if data:
                self.segments.append((seg["p_paddr"], data))
            if seg["p_flags"] & 1:  # PF_X
                self.code_ranges.append((seg["p_vaddr"], seg["p_vaddr"] + seg["p_memsz"]))
        funcs = {}
        sizes = {}
        for sec in elf.iter_sections():
            if not isinstance(sec, SymbolTableSection):
                continue
            for sym in sec.iter_symbols():
                name = sym.name
                if not name or name.startswith("$"):
                    continue
                st_type = sym["st_info"]["type"]
                addr = sym["st_value"]
                if st_type == "STT_FUNC":
                    addr &= ~1
                    # prefer global names over weak aliases for the same address
                    if addr not in funcs or sym["st_info"]["bind"] == "STB_GLOBAL":
                        funcs[addr] = name
                    sizes[addr] = max(sizes.get(addr, 0), sym["st_size"])
                # TI's linker marks uninitialised globals (int x;) as STT_COMMON
                if st_type in ("STT_FUNC", "STT_OBJECT", "STT_NOTYPE", "STT_COMMON"):
                    if name not in self.symbols or sym["st_info"]["bind"] == "STB_GLOBAL":
                        self.symbols[name] = addr
        for addr in sorted(funcs):
            self._func_addrs.append(addr)
            self._func_names.append(funcs[addr])
            self._func_sizes.append(sizes.get(addr, 0))

    @property
    def entry(self):
        return self._elf["e_entry"]

    def func_at(self, addr):
        """Return (name, offset) of the function containing addr, or (None, 0)."""
        i = bisect.bisect_right(self._func_addrs, addr) - 1
        if i < 0:
            return None, 0
        off = addr - self._func_addrs[i]
        size = self._func_sizes[i]
        if (size and off >= size) or off > 0x10000:
            return None, 0
        return self._func_names[i], off

    def _build_lines(self):
        self._lines = []
        try:
            if not self._elf.has_dwarf_info():
                return
            dwarf = self._elf.get_dwarf_info()
            for cu in dwarf.iter_CUs():
                lp = dwarf.line_program_for_CU(cu)
                if lp is None:
                    continue
                header = lp.header
                files = header["file_entry"]
                version = header.get("version", 2)
                for entry in lp.get_entries():
                    st = entry.state
                    if st is None or st.end_sequence:
                        continue
                    idx = st.file if version >= 5 else st.file - 1
                    name = "?"
                    if 0 <= idx < len(files):
                        raw = files[idx].name
                        name = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else str(raw)
                    self._lines.append((st.address, name, st.line))
        except Exception:
            pass  # TI DWARF quirks: fall back to symbols only
        self._lines.sort()
        self._line_addrs = [a for a, _, _ in self._lines]

    def line_at(self, addr):
        """Return 'File.c:123' for addr if debug info allows, else None."""
        if self._lines is None:
            self._build_lines()
        if not self._lines:
            return None
        i = bisect.bisect_right(self._line_addrs, addr) - 1
        if i < 0:
            return None
        a, name, line = self._lines[i]
        fname, off = self.func_at(addr)
        if not fname or a < addr - off:   # no function here, or the line entry belongs to another one
            return None
        short = name.replace("\\", "/").rsplit("/", 1)[-1]
        return f"{short}:{line}"

    def describe(self, addr):
        """Human readable location: 'Motor_Forward+0x12 (Motor.c:88)'."""
        name, off = self.func_at(addr)
        s = f"{name}+0x{off:x}" if name else f"0x{addr:08x}"
        line = self.line_at(addr)
        return f"{s} ({line})" if line else s

    def find_unmask_sites(self):
        """Addresses of CPSIE i and MSR PRIMASK/BASEPRI instructions: {addr: (kind, reg)}."""
        hits = {}
        for addr, data in self.segments:
            if not any(start <= addr < end for start, end in self.code_ranges):
                continue
            for off in range(0, len(data) - 1, 2):
                hw1 = data[off] | (data[off + 1] << 8)
                if hw1 == 0xB662:
                    hits[addr + off] = ("cpsie", None)
                elif hw1 & 0xFFF0 == 0xF380 and off + 3 < len(data):
                    hw2 = data[off + 2] | (data[off + 3] << 8)
                    if hw2 in (0x8810, 0x8811, 0x8812):
                        hits[addr + off] = ({0x8810: "primask", 0x8811: "basepri", 0x8812: "basepri_max"}[hw2], hw1 & 0xF)
        return hits

    def find_halfwords(self, value):
        """Addresses inside executable segments where the 16-bit value occurs (2-byte aligned)."""
        hits = []
        for addr, data in self.segments:
            for start, end in self.code_ranges:
                if start <= addr < end:
                    for off in range(0, len(data) - 1, 2):
                        if data[off] | (data[off + 1] << 8) == value:
                            hits.append(addr + off)
        return hits
