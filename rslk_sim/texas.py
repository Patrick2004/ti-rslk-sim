"""Decode the TExaS logic-analyzer / oscilloscope stream (inc/TExaS.c).

TExaS_Init() makes Timer32 interrupt 10,000 times a second and send one UART byte per
interrupt to TExaSdisplay on the PC: a port snapshot with bit 7 set (logic analyzer) or
an 8-bit ADC sample of P4.4/A9 (scope).  Instead of showing those bytes as junk text,
the simulator records them as samples and draws them like TExaSdisplay would.

The mode is found by reading TExaS's PeriodicTask2 pointer: it points at
LogicAnalyzer_P7(), Scope(), ...  Bytes are treated as samples only while that
Timer32 interrupt (IRQ 25) is enabled.
"""
import collections

T32_INT1_IRQ = 25
CHECK_EVERY = 0.005         # seconds of simulated time between mode checks

# function name -> channel labels for bits 0..n-1 of the sample
_SPECIAL = {
    "LogicAnalyzer_P4_765432": ["P4.2", "P4.3", "P4.4", "P4.5", "P4.6", "P4.7"],
    "LogicAnalyzer_P4_765320": ["P4.0", "P4.2", "P4.3", "P4.5", "P4.6", "P4.7"],
    "LogicAnalyzer_P2_7654": ["P2.4", "P2.5", "P2.6", "P2.7"],
    "LogicAnalyzer": [f"bit{i}" for i in range(7)],
}


def _labels_for(func):
    if func in _SPECIAL:
        return _SPECIAL[func]
    if func.startswith("LogicAnalyzer_P") and func[15:].isdigit():
        port = func[15:]
        return [f"P{port}.{i}" for i in range(7)]
    return None


class TexasMonitor:
    def __init__(self, machine, keep_all=False, history=2.0):
        self.m = machine
        prog = machine.prog
        self.ptr_addr = prog.symbols.get("PeriodicTask2")
        self.targets = {}
        if self.ptr_addr is not None:
            for name, addr in prog.symbols.items():
                if name == "Scope" or _labels_for(name):
                    self.targets[addr & ~1] = name
        self.mode = None            # None, "logic" or "scope"
        self.func = None
        self.labels = []
        self.samples = collections.deque(maxlen=int(history * 10000))   # (t, value)
        self.all = [] if keep_all else None
        self.count = 0
        self._next_check = 0.0

    @property
    def available(self):
        return bool(self.targets)

    def title(self):
        if self.mode == "scope":
            return "TExaS scope (P4.4/A9)"
        if self.mode == "logic":
            port = self.labels[0].split(".")[0] if self.labels and "." in self.labels[0] else ""
            return f"TExaS logic analyzer {port}".rstrip()
        return "TExaS"

    def _check_mode(self, t):
        self._next_check = t + CHECK_EVERY
        mode, func = None, None
        m = self.m
        if m.nvic.enabled >> T32_INT1_IRQ & 1:
            try:
                ptr = int.from_bytes(bytes(m.uc.mem_read(self.ptr_addr, 4)), "little") & ~1
            except Exception:
                ptr = 0
            func = self.targets.get(ptr)
            if func == "Scope":
                mode = "scope"
            elif func:
                mode = "logic"
        changed = func != self.func
        self.mode = mode
        if changed:
            self.func = func
            self.labels = ["A9"] if mode == "scope" else (_labels_for(func) or [])
            if mode:
                m.message(f"{self.title()} started (drawn as traces, not UART text)")

    def consume(self, byte, t):
        """Called for every UART0 byte; True if it was a TExaS sample (not text)."""
        if not self.targets:
            return False
        if t >= self._next_check or t < self._next_check - 2 * CHECK_EVERY or (self.mode is None and byte >= 0x80):
            self._check_mode(t)
        if self.mode is None or (self.mode == "logic" and byte < 0x80):
            return False
        v = byte & 0x7F if self.mode == "logic" else byte
        self.samples.append((t, v))
        if self.all is not None:
            self.all.append((t, v))
        self.count += 1
        return True

    # ------------------------------------------------------------ reporting
    def summary(self):
        """Per-channel statistics over the recorded samples (for headless runs)."""
        data = self.all if self.all is not None else list(self.samples)
        if not data:
            return []
        span = data[-1][0] - data[0][0] if len(data) > 1 else 0.0
        lines = [f"{self.title()}: {self.count} samples, statistics over the last {span:.2f} s"]
        if self.mode == "scope":
            vals = [v * 3.3 / 256 for _, v in data]
            lines.append(f"  A9  min {min(vals):.2f} V  max {max(vals):.2f} V  mean {sum(vals) / len(vals):.2f} V")
            return lines
        for bit, label in enumerate(self.labels):
            high = rises = 0
            prev = None
            for _, v in data:
                b = v >> bit & 1
                high += b
                if prev == 0 and b:
                    rises += 1
                prev = b
            freq = f"{rises / span:7.1f} Hz" if span > 0 else ""
            lines.append(f"  {label:5} high {100.0 * high / len(data):5.1f}%  rising edges {rises:6d}  {freq}")
        return lines

    def write_csv(self, path):
        data = self.all if self.all is not None else list(self.samples)
        with open(path, "w", encoding="utf-8") as f:
            if self.mode == "scope":
                f.write("time_s,volts\n")
                for t, v in data:
                    f.write(f"{t:.6f},{v * 3.3 / 256:.4f}\n")
            else:
                f.write("time_s," + ",".join(self.labels) + "\n")
                for t, v in data:
                    f.write(f"{t:.6f}," + ",".join(str(v >> b & 1) for b in range(len(self.labels))) + "\n")
        return len(data)
