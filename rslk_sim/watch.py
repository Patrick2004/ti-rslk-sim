"""Live view of the program's global and static variables (like CCS's Expressions window).

Variables are found in the DWARF debug info of the .out file (TI and GCC both emit it).
By default the watch list holds the variables defined in your own lab files: every
compilation unit that is not part of inc/, the startup/system files or the compiler
libraries.  Any other global can be added by name (``--watch LogicData``).
"""
import struct

_SKIP_FILES = ("startup_msp432p401r", "system_msp432p401r")


class CType:
    """A resolved C type: kind is int, uint, float, bool, char, ptr, enum, array, struct."""

    def __init__(self, kind, size, name="", elem=None, count=0, enums=None):
        self.kind, self.size, self.name = kind, size, name
        self.elem, self.count, self.enums = elem, count, enums or {}


class Variable:
    def __init__(self, name, addr, ctype, cu_file, func=None):
        self.name = name                     # plain C name
        self.addr = addr
        self.ctype = ctype
        self.cu_file = cu_file
        self.func = func                     # set for a function-local static
        self.label = f"{func}.{name}" if func else name

    def read(self, m):
        """Current value formatted for display, or '?' if it can't be read."""
        try:
            data = bytes(m.uc.mem_read(self.addr, min(self.ctype.size, 64)))
        except Exception:
            return "?"
        return format_value(self.ctype, data, m.prog)

    def raw(self, m):
        """Current value as a Python number (scalars only), else None."""
        t = self.ctype
        if t.kind not in ("int", "uint", "float", "bool", "char", "ptr", "enum") or t.size not in (1, 2, 4, 8):
            return None
        try:
            data = bytes(m.uc.mem_read(self.addr, t.size))
        except Exception:
            return None
        return _scalar(t, data)


def _scalar(t, data):
    if t.kind == "float":
        return struct.unpack("<f" if t.size == 4 else "<d", data[:t.size])[0]
    return int.from_bytes(data[:t.size], "little", signed=t.kind in ("int", "char") and t.name != "unsigned char")


def format_value(t, data, prog=None, compact=False):
    if t.kind == "array":
        if t.elem is None or t.elem.size == 0:
            return f"[{t.count}]"
        if t.elem.kind == "char":
            s = data[: t.count].split(b"\0", 1)[0]
            return '"' + s.decode("latin-1", "replace") + '"'
        n = min(t.count, len(data) // t.elem.size, 8)
        items = [format_value(t.elem, data[i * t.elem.size:(i + 1) * t.elem.size], prog, True) for i in range(n)]
        more = ", ..." if t.count > n else ""
        return "[" + ", ".join(items) + more + "]"
    if t.kind == "struct":
        return f"{{{t.name or 'struct'}, {t.size} bytes}}"
    if len(data) < t.size or t.size not in (1, 2, 4, 8):
        return "?"
    v = _scalar(t, data)
    if t.kind == "float":
        return f"{v:.6g}"
    if t.kind == "bool":
        return "true" if v else "false"
    if t.kind == "enum":
        return t.enums.get(v, str(v))
    if t.kind == "ptr":
        if prog is not None and v:
            name, off = prog.func_at(v & ~1)
            if name and off == 0:
                return f"0x{v:08x} <{name}>"
        return f"0x{v:08x}"
    if compact and t.kind in ("int", "uint", "char"):
        return str(v)
    if t.kind == "char" and t.size == 1 and 32 <= (v & 0xFF) < 127:
        return f"{v} '{chr(v & 0xFF)}'"
    if t.kind == "uint" and t.size == 1:
        return f"{v} (0x{v:02x}, {v:08b}b)"       # 8-bit values are usually bit patterns
    if t.kind == "uint":
        return f"{v} (0x{v:0{t.size * 2}x})"
    return str(v)


def _name(die):
    a = die.attributes.get("DW_AT_name")
    if a is None:
        return ""
    v = a.value
    return v.decode("utf-8", "replace") if isinstance(v, bytes) else str(v)


def _resolve(die, depth=0):
    if die is None or depth > 20:
        return CType("struct", 0)
    tag = die.tag
    a = die.attributes
    size = a["DW_AT_byte_size"].value if "DW_AT_byte_size" in a else 0
    if tag in ("DW_TAG_typedef", "DW_TAG_volatile_type", "DW_TAG_const_type", "DW_TAG_restrict_type"):
        if "DW_AT_type" not in a:
            return CType("struct", 0)
        inner = _resolve(die.get_DIE_from_attribute("DW_AT_type"), depth + 1)
        if tag == "DW_TAG_typedef" and inner.kind in ("int", "uint", "bool"):
            inner = CType(inner.kind, inner.size, inner.name or _name(die))
        return inner
    if tag == "DW_TAG_base_type":
        enc = a["DW_AT_encoding"].value if "DW_AT_encoding" in a else 7
        kind = {2: "bool", 4: "float", 5: "int", 6: "char", 7: "uint", 8: "uint"}.get(enc, "uint")
        return CType(kind, size, _name(die))
    if tag == "DW_TAG_pointer_type":
        return CType("ptr", size or 4)
    if tag == "DW_TAG_enumeration_type":
        enums = {}
        for c in die.iter_children():
            if c.tag == "DW_TAG_enumerator" and "DW_AT_const_value" in c.attributes:
                enums[c.attributes["DW_AT_const_value"].value] = _name(c)
        return CType("enum", size or 4, _name(die), enums=enums)
    if tag == "DW_TAG_array_type":
        elem = _resolve(die.get_DIE_from_attribute("DW_AT_type"), depth + 1) if "DW_AT_type" in a else None
        count = 1
        for c in die.iter_children():
            if c.tag == "DW_TAG_subrange_type":
                ca = c.attributes
                if "DW_AT_count" in ca:
                    count *= ca["DW_AT_count"].value
                elif "DW_AT_upper_bound" in ca and isinstance(ca["DW_AT_upper_bound"].value, int):
                    count *= ca["DW_AT_upper_bound"].value + 1
        return CType("array", size or (elem.size * count if elem else 0), elem=elem, count=count)
    if tag in ("DW_TAG_structure_type", "DW_TAG_union_type", "DW_TAG_class_type"):
        return CType("struct", size, _name(die))
    return CType("struct", size)


def _addr_of(die):
    loc = die.attributes.get("DW_AT_location")
    if loc is None or not isinstance(loc.value, (list, bytes)) or len(loc.value) != 5 or loc.value[0] != 0x03:
        return None                                    # only plain DW_OP_addr (statically allocated)
    return int.from_bytes(bytes(loc.value[1:5]), "little")


def is_user_file(path):
    p = path.replace("\\", "/").lower()
    if "/" not in p:
        return False             # compiler run-time library units are named without a folder (exit.c)
    if "/inc/" in p or p.startswith("inc/") or "/ti/ccs" in p or "/lib/" in p or p.endswith(".h"):
        return False
    return not any(s in p for s in _SKIP_FILES)


def find_variables(prog):
    """All statically allocated variables in the program: list of Variable."""
    found, seen = [], set()
    elf = prog._elf
    try:
        if not elf.has_dwarf_info():
            return found
        dwarf = elf.get_dwarf_info()
        for cu in dwarf.iter_CUs():
            top = cu.get_top_DIE()
            cu_file = _name(top)

            def walk(die, func):
                for c in die.iter_children():
                    if c.tag == "DW_TAG_variable":
                        addr = _addr_of(c)
                        name = _name(c)
                        if addr is not None and not name and "DW_AT_specification" in c.attributes:
                            name = _name(c.get_DIE_from_attribute("DW_AT_specification"))
                        if addr is None or not name or "$" in name or "DW_AT_type" not in c.attributes:
                            continue
                        key = (addr, name)
                        if key in seen:
                            continue
                        seen.add(key)
                        try:
                            ctype = _resolve(c.get_DIE_from_attribute("DW_AT_type"))
                        except Exception:
                            continue
                        found.append(Variable(name, addr, ctype, cu_file, func))
                    elif c.tag in ("DW_TAG_subprogram", "DW_TAG_lexical_block"):
                        walk(c, _name(c) if c.tag == "DW_TAG_subprogram" else func)

            walk(top, None)
    except Exception:
        pass                                           # unusual debug info: no watch list
    return found


class Watch:
    """The watch list shown in the window and printed by headless runs."""

    def __init__(self, prog, extra=()):
        self.all = find_variables(prog)
        self.items = [v for v in self.all if is_user_file(v.cu_file)]
        self.missing = []
        for name in extra:
            hits = [v for v in self.all if name in (v.label, v.name)]
            if not hits:
                self.missing.append(name)
            for v in hits:
                if v not in self.items:
                    self.items.append(v)

    def snapshot(self, m):
        return [(v.label, v.read(m)) for v in self.items]
