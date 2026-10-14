"""Command line: python -m rslk_sim <program.out> [options]"""
import argparse
import glob
import os
import sys
import time

WORLDS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "worlds")


def find_world(name):
    if not name:
        return None
    if os.path.exists(name):
        return name
    cand = os.path.join(WORLDS_DIR, name if name.endswith(".json") else name + ".json")
    if os.path.exists(cand):
        return cand
    sys.exit(f"world '{name}' not found (try --list-worlds)")


def printable(byte):
    if byte in (10, 13, 9) or 32 <= byte < 127:
        return chr(byte)
    return f"\\x{byte:02x}"


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m rslk_sim",
                                 description="Run a CCS-built TI-RSLK MAX program (.out) on a simulated robot.")
    ap.add_argument("program", nargs="?", help="the .out file CCS built (e.g. Lab2_FSM/Debug/Lab2_FSM.out)")
    ap.add_argument("--world", "-w", help="world JSON file or name from the worlds/ folder (default: empty floor)")
    ap.add_argument("--list-worlds", action="store_true", help="list the bundled worlds and exit")
    ap.add_argument("--headless", action="store_true", help="no window: run and print UART output")
    ap.add_argument("--sim-time", type=float, default=None,
                    help="seconds of robot time to simulate when headless (default 5; unlimited with --gdb)")
    ap.add_argument("--speed", type=float, default=1.0, help="initial speed relative to real time in the window (default 1)")
    ap.add_argument("--uart", action="append", default=[], metavar="[T:]TEXT",
                    help="type TEXT into the UART at simulated time T seconds (default 0.5); \\r and \\n are allowed; repeatable")
    ap.add_argument("--press", action="append", default=[], metavar="BUTTON@T1-T2",
                    help="hold SW1/SW2/BUMP0-5 from T1 to T2 seconds, e.g. SW1@1-1.2 (headless); repeatable")
    ap.add_argument("--watch", action="append", default=[], metavar="NAME[,NAME...]",
                    help="also show these global/static variables (your own lab file's variables are always shown); "
                         "a static inside a function is FUNCTION.NAME")
    ap.add_argument("--watch-every", type=float, default=None, metavar="SEC",
                    help="headless: print the watched variables every SEC seconds of robot time")
    ap.add_argument("--texas-csv", metavar="FILE",
                    help="headless: save the TExaS logic analyzer / scope samples to a CSV file")
    ap.add_argument("--cpi", type=float, default=2.0, help="average CPU cycles per instruction (default 2.0)")
    ap.add_argument("--no-hle", action="store_true", help="really execute Clock_Delay busy loops instead of skipping them")
    ap.add_argument("--no-idle-skip", action="store_true", help="don't fast-forward busy-wait polling loops")
    ap.add_argument("--trace-io", action="store_true", help="log every peripheral register access (very verbose)")
    ap.add_argument("--gdb", type=int, nargs="?", const=3333, default=None, metavar="PORT",
                    help="start a GDB server (default port 3333) for breakpoints / stepping from gdb or VS Code")
    ap.add_argument("--quiet", "-q", action="store_true", help="headless: only print UART output")
    ap.add_argument("--keep-running", action="store_true", help="headless: keep simulating after main() returns")
    ap.add_argument("--screenshot", metavar="PNG", help=argparse.SUPPRESS)
    ap.add_argument("--screenshot-time", type=float, default=2.0, help=argparse.SUPPRESS)
    args = ap.parse_args(argv)

    if args.list_worlds:
        import json
        for path in sorted(glob.glob(os.path.join(WORLDS_DIR, "*.json"))):
            with open(path, encoding="utf-8") as f:
                d = json.load(f)
            print(f"  {os.path.splitext(os.path.basename(path))[0]:<20} {d.get('description', d.get('name', ''))}")
        return 0
    if not args.program and not args.headless:
        args.program = pick_program()
    if not args.program:
        ap.error("the program (.out file) is required")
    if not os.path.exists(args.program):
        sys.exit(f"{args.program}: file not found")

    world = find_world(args.world)
    watch = [n.strip() for item in args.watch for n in item.split(",") if n.strip()]
    opts = dict(cpi=args.cpi, hle=not args.no_hle, idle_skip=not args.no_idle_skip, watch=watch)
    if args.headless:
        return run_headless(args, world, opts)
    from .ui import run_ui
    return run_ui(args, world, opts)


def pick_program():
    """Let the user choose the .out file in a file dialog (when started without arguments)."""
    try:
        import tkinter
        from tkinter import filedialog
        root = tkinter.Tk()
        root.withdraw()
        path = filedialog.askopenfilename(
            title="Choose the program CCS built (usually <project>/Debug/<project>.out)",
            filetypes=[("CCS / ARM programs", "*.out *.elf *.axf"), ("All files", "*.*")])
        root.destroy()
        return path or None
    except Exception:
        return None


def parse_uart_args(items):
    out = []
    for item in items:
        t = 0.5
        head, sep, rest = item.partition(":")
        if sep:
            try:
                t = float(head)
                item = rest
            except ValueError:
                pass
        text = item.encode("latin-1").decode("unicode_escape")
        out.append((t, text))
    return sorted(out)


def parse_press_args(items):
    out = []
    for item in items:
        name, _, span = item.partition("@")
        a, _, b = span.partition("-")
        out.append((name.upper(), float(a or 0), float(b or 1e9)))
    return out


def run_headless(args, world, opts):
    from .session import Session

    log = (lambda msg: None) if args.quiet else (lambda msg: print(msg, file=sys.stderr, flush=True))
    s = Session(args.program, world, log=log, texas_keep_all=bool(args.texas_csv), **opts)
    for name in s.watch.missing:
        print(f"--watch: no global or static variable named '{name}' in {os.path.basename(args.program)}",
              file=sys.stderr)
    next_watch = 0.0 if args.watch_every else None
    s.m.trace_io = args.trace_io
    stub = None
    if args.gdb:
        from .gdbstub import GdbStub
        stub = GdbStub(s, args.gdb, log=lambda msg: print(msg, file=sys.stderr, flush=True))
        print("waiting for the debugger to connect (program is paused at reset)...", file=sys.stderr, flush=True)
    sim_time = args.sim_time if args.sim_time is not None else (float("inf") if stub else 5.0)
    was_connected = False
    s.uart_listeners.append(lambda b, t: (sys.stdout.write(printable(b)), sys.stdout.flush()))
    uart_events = parse_uart_args(args.uart)
    presses = parse_press_args(args.press)
    t_wall = time.perf_counter()
    step = 0.01
    finished_at = None
    while s.m.t < sim_time and not s.m.halted:
        if stub:
            stub.poll()
            if stub.connected:
                was_connected = True
            elif was_connected:
                break                           # debugger detached: end the session
            if stub.halted or not stub.connected:
                time.sleep(0.003)
                continue
        if s.m.finished:
            finished_at = s.m.t if finished_at is None else finished_at
            if s.m.t > finished_at + 0.25 and not args.keep_running:
                break                           # let the UART drain, then stop
        while uart_events and uart_events[0][0] <= s.m.t:
            s.m.uart_input(uart_events.pop(0)[1])
        bumps = 0
        for name, a, b in presses:
            down = a <= s.m.t < b
            if name in ("SW1", "SW2"):
                s.robot.press_switch(name, down)
            elif name.startswith("BUMP") and down:
                bumps |= 1 << int(name[4:])
        s.robot.set_forced_bumps(bumps)
        if next_watch is not None and s.m.t >= next_watch - 1e-9:
            print_watch(s, sys.stdout)
            next_watch += args.watch_every
        run_step = step if next_watch is None else max(1e-4, min(step, next_watch - s.m.t))
        s.run_for(min(run_step, sim_time - s.m.t))
        if stub and s.m.debug_halt:
            stub.machine_stopped_at_breakpoint()
    wall = time.perf_counter() - t_wall
    r = s.robot
    if not args.quiet:
        print("\n" + "-" * 60, file=sys.stderr)
        print(f"simulated {s.m.t:.3f} s in {wall:.2f} s wall time ({s.m.t / wall if wall else 0:.1f}x real time)", file=sys.stderr)
        print(f"MCLK {s.m.clk['MCLK'] / 1e6:g} MHz, {s.m.instructions:,} instructions executed", file=sys.stderr)
        print(f"robot at x={r.x:.3f} m y={r.y:.3f} m heading={r.th * 57.2958:.1f} deg, travelled {r.distance:.3f} m", file=sys.stderr)
        print(f"program counter: {s.m.describe_pc(s.m.pc)}", file=sys.stderr)
        if s.m.halted:
            print(f"STOPPED: {s.m.halted}", file=sys.stderr)
        if s.m.finished:
            print("main() returned", file=sys.stderr)
        if s.watch.items:
            print("variables at the end:", file=sys.stderr)
            for label, value in s.watch.snapshot(s.m):
                print(f"  {label:<28} {value}", file=sys.stderr)
        for line in s.texas.summary():
            print(line, file=sys.stderr)
    if args.texas_csv:
        n = s.texas.write_csv(args.texas_csv)
        print(f"wrote {n} TExaS samples to {args.texas_csv}", file=sys.stderr)
    return 1 if s.m.halted else 0


def short_value(value):
    """'24 (0x18, 00011000b)' -> '0x18' for compact periodic lines; other values unchanged."""
    if " (0x" in value:
        return value.split(" (")[1].split(",")[0].rstrip(")")
    return value


def print_watch(s, out):
    items = s.watch.snapshot(s.m)
    if items:
        out.write(f"[{s.m.t:9.4f}s] " + "  ".join(f"{label}={short_value(value)}" for label, value in items) + "\n")
        out.flush()


if __name__ == "__main__":
    sys.exit(main())
