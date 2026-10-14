import os
import re
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

# TI's RSLK-MAX-SelfTest.out (part of the TI-RSLK MAX curriculum, not redistributed here).
# Point RSLK_SELFTEST at your copy to run tests/test_selftest.py; otherwise those tests are skipped.
SELFTEST = os.environ.get("RSLK_SELFTEST", os.path.join(ROOT, "tests", "data", "RSLK-MAX-SelfTest.out"))
WORLDS = os.path.join(ROOT, "worlds")


@pytest.fixture(scope="session")
def fw():
    """Build fw_tests/*.c once per test session; returns a function name -> elf path."""
    import build_fw
    ccs = build_fw.find_ccs()
    if not ccs:
        pytest.skip("CCS with GCC ARM not installed - firmware tests skipped")
    assert build_fw.build(ccs, quiet=True), "firmware build failed"
    return lambda name: os.path.join(ROOT, "fw_tests", "build", name + ".elf")


def run(elf, world=None, seconds=1.0, uart=(), presses=(), step=0.01, setup=None):
    """Run a program headless.  uart: [(time, text)], presses: [(name, t0, t1)]."""
    from rslk_sim.session import Session
    wpath = os.path.join(WORLDS, world + ".json") if world else None
    s = Session(elf, wpath, log=lambda m: None)
    if setup:
        setup(s)
    uart = sorted(uart)
    while s.m.t < seconds and not s.m.halted:
        while uart and uart[0][0] <= s.m.t:
            s.m.uart_input(uart.pop(0)[1])
        for name, a, b in presses:
            down = a <= s.m.t < b
            if name.startswith("SW"):
                s.robot.press_switch(name, down)
            elif name.startswith("BUMP"):
                bit = 1 << int(name[4:])
                s.robot.set_forced_bumps((s.robot.forced_bumps | bit) if down else (s.robot.forced_bumps & ~bit))
        s.run_for(min(step, seconds - s.m.t))
    return s, s.uart_text.decode("latin-1")


def values(text, key):
    """All integer values printed as key=NNN."""
    return [int(v) for v in re.findall(rf"\b{key}=(-?\d+)", text)]
