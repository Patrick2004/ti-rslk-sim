"""TI's RSLK MAX self-test program (CCS build of the real TI drivers) on the simulator."""
import os

import pytest

from conftest import SELFTEST, run

pytestmark = pytest.mark.skipif(not os.path.exists(SELFTEST), reason="RSLK-MAX-SelfTest.out not found")


def test_menu_appears():
    s, out = run(SELFTEST, seconds=0.3)
    assert "RSLK Testing" in out and "[5] Tachometer Test" in out and out.rstrip().endswith("CMD:")
    assert s.m.clk["MCLK"] == 48e6 and not s.m.halted


def test_reflectance_sees_line():
    s, out = run(SELFTEST, world="line_oval", seconds=1.0, uart=[(0.3, "4\r")])
    assert "Reflectance Sensor Data = 0x18" in out          # the two centre sensors are on the tape


def test_bumper_switches():
    s, out = run(SELFTEST, seconds=2.2, uart=[(0.3, "3\r")],
                 presses=[("BUMP2", 1.0, 1.2), ("BUMP5", 1.5, 1.7)])
    assert "Bumper Switch 3" in out and "Bumper Switch 6" in out


def test_motor_test_drives_forward_and_back():
    s, out = run(SELFTEST, seconds=7.0, uart=[(0.3, "1\r"), (1.0, "x"), (4.0, "x")])
    assert "Motor Backward" in out
    assert s.robot.distance > 0.8 and abs(s.robot.y - 1.0) < 0.02


def test_tachometer_periods_settle():
    s, out = run(SELFTEST, seconds=3.0, uart=[(0.3, "5\r")])
    periods = [int(p, 16) for p in __import__("re").findall(r"Period0 = 0x([0-9A-F]+)", out)]
    assert len(periods) > 20
    last = periods[-10:]
    assert max(last) - min(last) <= 2                        # steady speed -> steady capture period
    assert s.robot.distance > 0.3
