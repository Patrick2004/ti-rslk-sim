"""Unit tests for peripheral models, driven through register writes without any program."""
import math

import pytest

from rslk_sim.periph.timer_a import TimerA


class FakeMachine:
    """Just enough of Machine for peripherals that are exercised directly."""

    def __init__(self):
        from rslk_sim.sched import Scheduler
        self.sched = Scheduler()
        self.t = 0.0
        self.clk = {"MCLK": 48e6, "HSMCLK": 24e6, "SMCLK": 12e6, "ACLK": 32768.0, "BCLK": 32768.0}
        self.irq_events = 0

    def now(self):
        return self.t

    def mark_dynamic(self):
        pass

    def irq_maybe_changed(self):
        self.irq_events += 1

    def pwm_changed(self, t):
        pass

    def warn_once(self, key, text):
        pass

    def run_events(self, until):
        while True:
            ev = self.sched.pop_due(until)
            if ev is None:
                break
            self.t = ev.t
            ev.fn(ev.t)
        self.t = until


def w16(p, off, v):
    p.write(off, 2, v)


def test_timer_a_updown_pwm_duty_matches_valvano_formula():
    m = FakeMachine()
    ta = TimerA(m, 0x40000000, 0)
    w16(ta, 0x12, 15000)          # CCR0 period
    w16(ta, 0x08, 0x40)           # CCTL3 OUTMOD 2 toggle/reset
    w16(ta, 0x18, 3750)           # CCR3
    w16(ta, 0x00, 0x02F0)         # SMCLK /8 up/down
    assert ta.duty(3) == pytest.approx(0.25)
    w16(ta, 0x18, 0)
    assert ta.duty(3) == pytest.approx(0.0)
    w16(ta, 0x18, 15000)
    assert ta.duty(3) == pytest.approx(1.0, abs=1e-3)


def test_timer_a_up_mode_reset_set_duty():
    m = FakeMachine()
    ta = TimerA(m, 0x40000000, 0)
    w16(ta, 0x12, 999)            # period 1000 ticks
    w16(ta, 0x04, 0xE0)           # CCTL1 OUTMOD 7 reset/set
    w16(ta, 0x14, 300)
    w16(ta, 0x00, 0x0210)         # SMCLK up mode
    assert ta.duty(1) == pytest.approx(0.3)


def test_timer_a_periodic_flag_and_iv():
    m = FakeMachine()
    ta = TimerA(m, 0x40000000, 1)
    w16(ta, 0x12, 11999)          # 1 ms at 12 MHz
    w16(ta, 0x00, 0x0216 | 0x02)  # SMCLK, up, clear, TAIE
    m.run_events(0.00105)
    assert ta.read(0x00, 2) & 1                          # TAIFG set after one period
    assert ta.read(0x2E, 2) == 0x0E                      # TAxIV reports TAIFG ...
    assert not ta.read(0x00, 2) & 1                      # ... and reading it clears the flag


def test_timer_a_capture():
    m = FakeMachine()
    ta = TimerA(m, 0x40000C00, 3)
    w16(ta, 0x00, 0x0224)                                # SMCLK continuous, clear
    w16(ta, 0x02, 0x4910)                                # CCTL0 rising-edge capture, CCIE
    m.t = 0.001
    ta.input_edge(0, True, 0.001)
    assert ta.ccr[0] == 12000 and ta.cctl[0] & 1
    ta.input_edge(0, True, 0.002)
    assert ta.cctl[0] & 0x02                             # overflow (COV) because CCIFG still set


def test_systick_countflag_and_value():
    from rslk_sim.periph.core import SysTick
    m = FakeMachine()
    m.nvic = type("N", (), {"set_pending": lambda self, e: None})()
    st = SysTick(m)
    st.write(0x14, 47999)          # 1 ms
    st.write(0x18, 0)
    st.write(0x10, 0x5)            # enable, MCLK, no interrupt
    m.run_events(0.0005)
    assert st.read(0x10) & 0x10000 == 0
    assert 23000 < st.read(0x18) < 25000
    m.run_events(0.00101)
    assert st.read(0x10) & 0x10000                       # COUNTFLAG set...
    assert st.read(0x10) & 0x10000 == 0                  # ...and cleared by reading CTRL


def test_gpio_edges_and_iv(fw):
    from rslk_sim.elfload import Program
    from rslk_sim.machine import Machine
    m = Machine(Program(fw("t_uart_systick")))           # any MSP432 program will do
    g = m.gpio
    state = {"v": 0}
    g.attach_input(4, lambda t: (0x01, state["v"]))   # external device drives P4.0
    g.write(0x21, 1, 0x00)    # P4DIR = inputs  (P4 is the high byte of pair PB at 0x20)
    g.write(0x39, 1, 0x01)    # P4IES: falling edge
    g.write(0x3D, 1, 0x00)    # P4IFG clear
    g.write(0x3B, 1, 0x01)    # P4IE
    state["v"] = 1
    g.ext_changed(4, 0.0)     # rising: no flag (IES=1)
    assert g.ifg[4] == 0
    state["v"] = 0
    g.ext_changed(4, 0.0)     # falling: flag
    assert g.ifg[4] == 1 and g.irq_line(4)
    assert g.read(0x3E, 2) == 0x02 and g.ifg[4] == 0   # P4IV = 2 for bit 0, read clears


def test_robot_sharp_curve():
    from rslk_sim.robot.robot import Robot
    assert Robot.sharp_volts(0.10) == pytest.approx(2.3)
    assert Robot.sharp_volts(0.30) == pytest.approx(0.92)
    assert Robot.sharp_volts(0.20) > Robot.sharp_volts(0.40) > Robot.sharp_volts(0.80)
