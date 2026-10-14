"""End-to-end tests: real ARM programs (built with CCS's GCC) on the simulated robot."""
import math

from conftest import run, values


def test_uart_systick_and_input(fw):
    s, out = run(fw("t_uart_systick"), seconds=1.5, uart=[(0.8, "abc\r")])
    assert out.startswith("hello\r\n")
    assert [f"tick {i}" in out for i in range(5)] == [True] * 5
    assert "type> abc\r\ndone\r\n" in out
    assert s.m.finished and not s.m.halted
    assert s.m.clk["MCLK"] == 48e6


def test_uart_timing_is_realistic(fw):
    # 5 x 100 ms of SysTick delays must take ~0.5 s of simulated time
    s, out = run(fw("t_uart_systick"), seconds=0.45)
    assert "tick 3" in out and "tick 4" not in out


def test_interrupt_rates_priorities_and_fpu(fw):
    s, out = run(fw("t_interrupts"), seconds=1.5)
    assert 998 <= values(out, "ta1")[0] <= 1001
    assert 99 <= values(out, "systick")[0] <= 101
    assert 249 <= values(out, "t32")[0] <= 251
    assert values(out, "isr_acc")[0] == 2500          # float math inside the ISR
    assert values(out, "main_ok")[0] == 1             # ... does not corrupt main's float state


def test_line_follower_stays_on_oval(fw):
    trace = []

    def setup(s):
        def sample(t):
            r = s.robot
            # distance from the oval centre line (straights y=0.35/1.15, half circles r=0.4)
            if 0.55 <= r.x <= 1.45:
                d = min(abs(r.y - 0.35), abs(r.y - 1.15))
            else:
                cx = 0.55 if r.x < 0.55 else 1.45
                d = abs(math.hypot(r.x - cx, r.y - 0.75) - 0.40)
            trace.append(d)
            s.m.sched.at(t + 0.05, sample, owner="world")
        s.m.sched.at(0.5, sample, owner="world")

    s, out = run(fw("t_line_follow"), world="line_oval", seconds=20.0, setup=setup)
    assert s.robot.distance > 2.0
    assert values(out, "lost")[-1] == 0
    assert max(trace) < 0.03                             # wheel axle within 3 cm of the line


def test_tachometer_capture_and_bump_interrupt(fw):
    s, out = run(fw("t_bump_tach"), world="wall_arena", seconds=8.0)
    rpm_l, rpm_r = values(out, "rpm_l"), values(out, "rpm_r")
    model_rpm = s.robot.p.wheel_rpm_full * (0.5 - s.robot.p.deadband) / (1 - s.robot.p.deadband)
    assert abs(rpm_l[-1] - model_rpm) <= 2 and abs(rpm_r[-1] - model_rpm) <= 2
    steps = values(out, "steps_l")
    assert steps == sorted(steps[:-1]) + [steps[-1]]     # counts up while driving forward
    assert values(out, "bump")[0] & 0b001100              # front-centre switches hit the wall
    assert steps[-1] < steps[-2]                          # backing up counts down
    assert 1.4 < s.robot.y < 1.55                         # stopped at the north wall


def test_adc_bitband_and_software_pwm(fw):
    def face_wall(s):
        s.robot.set_pose(0.8, 1.6 - 0.25, math.pi / 2)   # centre sensor ~19 cm from the wall
    s, out = run(fw("t_adc_softpwm"), world="wall_arena", seconds=0.5, setup=face_wall)
    c = values(out, "adc_c")[0] * 3.3 / 16383
    assert 1.2 < c < 1.6, c                               # GP2Y0A21 gives ~1.35 V at 19 cm
    assert values(out, "led1") == [1]
    s, out = run(fw("t_adc_softpwm"), world="wall_arena", seconds=3.0)
    assert "softpwm done" in out
    assert 0.15 < s.robot.distance < 0.4                  # 25 % duty software PWM for 2 s


def test_null_pointer_fault_names_source_line(fw):
    s, out = run(fw("t_faults"), seconds=0.5, uart=[(0.1, "n")])
    assert s.m.halted and "NULL pointer" in s.m.halted and "t_faults.c:15" in s.m.halted


def test_missing_interrupt_handler_is_reported(fw):
    s, out = run(fw("t_faults"), seconds=0.5, uart=[(0.1, "h")])
    assert s.m.halted and "TA2_0_IRQHandler" in s.m.halted


def test_watchdog_reset_reboots(fw):
    s, out = run(fw("t_faults"), seconds=0.5, uart=[(0.1, "w")])
    assert out.count("boot") == 2 and s.m.resets == 1


def test_exact_delay_execution_matches_hle(fw):
    """Clock_Delay1ms skipping (HLE) and real execution give similar timing."""
    from rslk_sim.session import Session
    times = []
    for hle in (True, False):
        s = Session(fw("t_uart_systick"), None, log=lambda m: None, hle=hle)
        s.run_for(0.2)
        times.append(s.m.t)
        assert "tick 0" in s.uart_text.decode()
    assert abs(times[0] - times[1]) < 1e-6


def test_texas_stream_and_watch(fw):
    """TExaS logic-analyzer bytes become samples (not text); the watch list decodes C types."""
    s, out = run(fw("t_texas"), seconds=0.2)
    assert out == "before\r\nafter\r\n"                  # no sample bytes leak into the text
    tx = s.texas
    assert tx.title() == "TExaS logic analyzer P7" and tx.labels[0] == "P7.0"
    assert 480 <= tx.count <= 520                        # 10 kHz for 50 ms
    p70 = [v & 1 for _, v in tx.samples]
    assert 0.4 < sum(p70) / len(p70) < 0.6               # 1 ms high / 1 ms low square wave
    assert all(v & 0x04 == 0 for _, v in tx.samples)     # P7.2 stays low
    vals = dict(s.watch.snapshot(s.m))
    assert vals["temperature"] == "-42"
    assert vals["gain"] == "1.5"
    assert vals["table"] == "[10, 20, 30, 40]"
    assert vals["state"] == "DONE"
    assert vals["main.ticks"].startswith("50 ")
    assert vals["PeriodicTask2"].endswith("<LogicAnalyzer_P7>")
