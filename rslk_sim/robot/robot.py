"""The simulated TI-RSLK MAX robot: motors, wheels, encoders, bump switches,
line sensor, IR distance sensors and LaunchPad buttons, wired to a Machine."""
import math
import random

from . import pins

PHYS_DT = 0.001          # physics step, seconds
IR_PERIOD = 0.0383       # Sharp GP2Y0A21 measurement cycle


class RobotParams:
    wheel_radius = 0.035      # m (70 mm wheels)
    track = 0.141             # m between wheel contact points
    body_radius = 0.083       # m (Romi chassis is ~165 mm across)
    battery = 7.2             # V (6 x AA NiMH)
    wheel_rpm_full = 150.0    # wheel speed at 100 % duty, no load
    motor_tau = 0.08          # s, speed response time constant (driven / braking)
    coast_tau = 0.35          # s, slow-down when the drivers are asleep
    deadband = 0.06           # duty fraction needed to overcome friction
    encoder_cpr = 360         # channel-A pulses per wheel revolution
    sensor_bar_x = 0.070      # m, line sensor distance in front of the axle
    t_white = 0.25e-3         # s, QTRX decay over white
    t_black = 2.5e-3          # s, QTRX decay over black tape
    t_dark = 5.0e-3           # s, decay with the IR LEDs off
    ir_noise = 0.01           # V rms
    seed = 1


class Motor:
    def __init__(self, robot, cfg):
        self.r = robot
        self.cfg = cfg
        self.acc = 0.0            # integral of drive over the current physics step
        self.acc_t = 0.0
        self.drive = 0.0          # current signed drive (-1..1)
        self.asleep = True
        self.omega = 0.0          # wheel speed, rad/s (+ = forward)
        self.phi = 0.0            # wheel angle, rad
        self.avg_drive = 0.0

    def inputs_changed(self, t):
        self._integrate(t)
        m = self.r.m
        port, bit = self.cfg["sleep"]
        awake = m.pin_drive(port, bit)
        self.asleep = not awake or awake < 0.5
        en = m.pin_drive(*self.cfg["pwm"]) or 0.0
        d = m.pin_drive(*self.cfg["dir"]) or 0.0
        self.drive = 0.0 if self.asleep else (-en if d >= 0.5 else en)

    def _integrate(self, t):
        if t > self.acc_t:
            self.acc += self.drive * (t - self.acc_t)
            self.acc_t = t

    def step(self, t, dt, p):
        self._integrate(t)
        avg = self.acc / dt if dt > 0 else self.drive
        self.acc = 0.0
        self.avg_drive = avg
        wmax = p.wheel_rpm_full * 2 * math.pi / 60.0 * (p.battery / 7.2)
        mag = abs(avg)
        eff = 0.0 if mag < p.deadband else (mag - p.deadband) / (1.0 - p.deadband)
        target = math.copysign(eff, avg) * wmax
        if self.asleep:
            target, tau = 0.0, p.coast_tau
        else:
            tau = p.motor_tau
        self.omega += (target - self.omega) * (1.0 - math.exp(-dt / tau))
        if abs(self.omega) < 1e-4 and target == 0.0:
            self.omega = 0.0


class Encoder:
    """Quadrature encoder.  Edges are scheduled as exact-time events."""

    def __init__(self, robot, motor, cfg, b_high_forward):
        self.r = robot
        self.motor = motor
        self.cfg = cfg
        self.b_sign = 0.25 if b_high_forward else -0.25
        self.a = self.level_a(0.0)
        self.b = self.level_b(0.0)

    def level_a(self, c):
        return 1 if (c % 1.0) < 0.5 else 0

    def level_b(self, c):
        return 1 if ((c + self.b_sign) % 1.0) < 0.5 else 0

    def schedule(self, t0, c0, rate, dt):
        """Schedule A and B edges for c(t) = c0 + rate*(t-t0) over (t0, t0+dt]."""
        if rate == 0:
            return
        c1 = c0 + rate * dt
        sched = self.r.m.sched
        for chan, offset in (("a", 0.0), ("b", self.b_sign)):
            lo, hi = sorted((2 * (c0 + offset), 2 * (c1 + offset)))
            k = math.floor(lo) + 1
            while k <= hi:
                cross = k / 2.0 - offset
                te = t0 + (cross - c0) / rate
                if te > t0:
                    level = (k % 2 == 0) if rate > 0 else (k % 2 == 1)
                    sched.at(te, lambda tt, ch=chan, lv=int(level): self._edge(ch, lv, tt), owner="world")
                k += 1

    def _edge(self, chan, level, t):
        m = self.r.m
        port, bit = self.cfg[chan]
        if chan == "a":
            if level == self.a:
                return
            self.a = level
            m.gpio.ext_changed(port, t)
            m.pin_edge(port, bit, bool(level), t)
        else:
            if level == self.b:
                return
            self.b = level
            m.gpio.ext_changed(port, t)


class Robot:
    def __init__(self, machine, world, params=None):
        self.m = machine
        self.world = world
        self.p = params or RobotParams()
        self.rng = random.Random(self.p.seed)
        self.x, self.y, self.th = world.start
        self.left = Motor(self, pins.LEFT_MOTOR)
        self.right = Motor(self, pins.RIGHT_MOTOR)
        bhf = pins.ENCODER_B_HIGH_WHEN_FORWARD
        self.enc_l = Encoder(self, self.left, pins.LEFT_ENCODER, bhf["left"])
        self.enc_r = Encoder(self, self.right, pins.RIGHT_ENCODER, bhf["right"])
        self.bumps = 0                  # bit i set = bump switch i pressed (by a wall)
        self.forced_bumps = 0           # pressed from the UI
        self.switches = {"SW1": False, "SW2": False}
        self.ir_volts = {k: 0.3 for k in pins.IR_SENSORS}
        self.ir_dist = {k: 1.5 for k in pins.IR_SENSORS}
        self._next_ir = 0.0
        self.stalled = False
        self.distance = 0.0             # odometer, metres
        # reflectance sensor state per bit
        self.refl_charged = [False] * 8
        self.refl_release = [0.0] * 8
        self.refl_tau = [0.0] * 8
        self.refl_events = [None] * 8
        self.refl_prev_dir = 0
        self.refl_seen = [1.0] * 8      # floor brightness under each sensor at last read
        self._wire()
        self._update_ir()
        self._step_ev = self.m.sched.at(self.m.t + PHYS_DT, self._step, owner="world")

    # ------------------------------------------------------------- wiring
    def _wire(self):
        m, g = self.m, self.m.gpio
        for cfg in (pins.LEFT_MOTOR, pins.RIGHT_MOTOR):
            for key in ("pwm", "dir", "sleep"):
                port = cfg[key][0]
                if self._motor_changed not in g.listeners[port]:
                    g.on_output(port, self._motor_changed)
        m.pwm_listeners.append(self._motor_changed)
        # inputs
        bump_ports = {p for p, _, _ in pins.BUMP_SWITCHES}
        for port in bump_ports:
            g.attach_input(port, lambda t, port=port: self._bump_input(port))
        g.attach_input(pins.SW1[0], lambda t: self._switch_input(pins.SW1[0]))
        if pins.SW2[0] != pins.SW1[0]:
            g.attach_input(pins.SW2[0], lambda t: self._switch_input(pins.SW2[0]))
        enc_ports = {self.enc_l.cfg["a"][0], self.enc_l.cfg["b"][0], self.enc_r.cfg["a"][0], self.enc_r.cfg["b"][0]}
        for port in enc_ports:
            g.attach_input(port, lambda t, port=port: self._encoder_input(port))
        g.attach_input(pins.REFLECT_PORT, self._reflect_input)
        g.on_output(pins.REFLECT_PORT, self._reflect_outputs_changed)
        for name, (pin, *_rest) in pins.IR_SENSORS.items():
            for p in [pin] + list(getattr(pins, "IR_EXTRA_PINS", {}).get(name, ())):
                m.analog_sources[p] = lambda t, n=name: self._ir_output(n)
        m.reset_listeners.append(self._on_mcu_reset)
        self._motor_changed(m.now())

    def _on_mcu_reset(self, t):
        self._motor_changed(t)
        self.refl_prev_dir = 0
        for i in range(8):
            self._cancel_refl(i)
            self.refl_charged[i] = False

    # ------------------------------------------------------------- inputs
    def _bump_input(self, port):
        pressed = self.bumps | self.forced_bumps
        mask = 0
        for i, (p, bit, _) in enumerate(pins.BUMP_SWITCHES):
            if p == port and pressed >> i & 1:
                mask |= 1 << bit
        return mask, 0                   # pressed switches pull the pin to ground

    def _switch_input(self, port):
        mask = 0
        for name, (p, bit) in (("SW1", pins.SW1), ("SW2", pins.SW2)):
            if p == port and self.switches[name]:
                mask |= 1 << bit
        return mask, 0

    def _encoder_input(self, port):
        mask = val = 0
        for enc in (self.enc_l, self.enc_r):
            for chan, level in (("a", enc.a), ("b", enc.b)):
                p, bit = enc.cfg[chan]
                if p == port:
                    mask |= 1 << bit
                    if level:
                        val |= 1 << bit
        return mask, val

    def press_switch(self, name, down):
        if self.switches.get(name) == down:
            return
        self.switches[name] = down
        port = pins.SW1[0] if name == "SW1" else pins.SW2[0]
        self.m.gpio.ext_changed(port, self.m.t)

    def set_forced_bumps(self, mask):
        if mask != self.forced_bumps:
            self.forced_bumps = mask
            for port in {p for p, _, _ in pins.BUMP_SWITCHES}:
                self.m.gpio.ext_changed(port, self.m.t)

    # --------------------------------------------------------- reflectance
    def _sensor_world_pos(self, bit):
        c, s = math.cos(self.th), math.sin(self.th)
        fx, fy = self.p.sensor_bar_x, pins.REFLECT_Y[bit]
        return self.x + c * fx - s * fy, self.y + s * fx + c * fy

    def floor_under_sensor(self, bit):
        x, y = self._sensor_world_pos(bit)
        w = self.world
        # average a small footprint (~4 mm) so line edges give intermediate values
        d = 0.002
        return (w.reflectance(x, y) * 2 + w.reflectance(x + d, y) + w.reflectance(x - d, y)
                + w.reflectance(x, y + d) + w.reflectance(x, y - d)) / 6.0

    def _led_on(self, bit):
        led = pins.REFLECT_LED_EVEN if bit % 2 == 1 else pins.REFLECT_LED_ODD
        return self.m.gpio.is_output_high(*led)

    def _cancel_refl(self, bit):
        ev = self.refl_events[bit]
        if ev:
            ev.cancel()
            self.refl_events[bit] = None

    def _reflect_outputs_changed(self, t):
        g = self.m.gpio
        port = pins.REFLECT_PORT
        d = g.dir[port]
        o = g.out[port]
        changed = d ^ self.refl_prev_dir
        p = self.p
        for bit in range(8):
            b = 1 << bit
            if d & b:
                # driven by the MCU: charges (high) or discharges (low) the capacitor
                self._cancel_refl(bit)
                self.refl_charged[bit] = bool(o & b)
            elif changed & b and self.refl_charged[bit]:
                # released to input: capacitor decays at a rate set by the reflected IR light
                brightness = self.floor_under_sensor(bit)
                self.refl_seen[bit] = brightness
                if self._led_on(bit):
                    tau = p.t_white + (p.t_black - p.t_white) * (1.0 - brightness)
                else:
                    tau = p.t_dark
                tau *= 1.0 + 0.03 * self.rng.uniform(-1, 1)
                self.refl_release[bit] = t
                self.refl_tau[bit] = tau
                self._cancel_refl(bit)
                self.refl_events[bit] = self.m.sched.at(t + tau, lambda tt, bb=bit: self._refl_decayed(bb, tt), owner="world")
        self.refl_prev_dir = d
        if changed:
            g.ext_changed(port, t)

    def _refl_decayed(self, bit, t):
        self.refl_events[bit] = None
        self.refl_charged[bit] = False
        self.m.gpio.ext_changed(pins.REFLECT_PORT, t)

    def _reflect_input(self, t):
        val = 0
        for bit in range(8):
            if self.refl_charged[bit]:
                val |= 1 << bit
        return 0xFF, val

    # ------------------------------------------------------------ IR sensors
    @staticmethod
    def sharp_volts(d):
        """GP2Y0A21YK0F output voltage for distance d in metres."""
        cm = d * 100.0
        table = [(4, 2.0), (6, 3.1), (8, 2.75), (10, 2.3), (15, 1.65), (20, 1.3), (30, 0.92),
                 (40, 0.75), (50, 0.6), (60, 0.5), (70, 0.45), (80, 0.4), (100, 0.35), (150, 0.3)]
        if cm <= table[0][0]:
            return table[0][1] * cm / table[0][0]
        for (d0, v0), (d1, v1) in zip(table, table[1:]):
            if cm <= d1:
                return v0 + (v1 - v0) * (cm - d0) / (d1 - d0)
        return table[-1][1]

    def _update_ir(self):
        c, s = math.cos(self.th), math.sin(self.th)
        for name, (_pin, mx, my, ang) in pins.IR_SENSORS.items():
            x = self.x + c * mx - s * my
            y = self.y + s * mx + c * my
            d = self.world.raycast(x, y, self.th + math.radians(ang), 1.5)
            self.ir_dist[name] = d
            self.ir_volts[name] = max(0.0, self.sharp_volts(d) + self.rng.gauss(0, self.p.ir_noise))

    def _ir_output(self, name):
        return self.ir_volts[name]

    # --------------------------------------------------------------- motion
    def _motor_changed(self, t):
        self.left.inputs_changed(t)
        self.right.inputs_changed(t)

    def set_pose(self, x, y, th):
        """Move the robot (e.g. dragged in the window); sensors update immediately."""
        self.x, self.y, self.th = x, y, (th + math.pi) % (2 * math.pi) - math.pi
        self._update_ir()
        self._update_bumps(self.m.now())

    def _step(self, t):
        p = self.p
        dt = PHYS_DT
        # 1. move with the wheel speeds of the interval that just ended
        wl, wr = self.left.omega, self.right.omega
        self.left.phi += wl * dt
        self.right.phi += wr * dt
        v = p.wheel_radius * (wl + wr) / 2.0
        w = p.wheel_radius * (wr - wl) / p.track
        if abs(w) > 1e-9:
            nx = self.x + v / w * (math.sin(self.th + w * dt) - math.sin(self.th))
            ny = self.y - v / w * (math.cos(self.th + w * dt) - math.cos(self.th))
        else:
            nx = self.x + v * dt * math.cos(self.th)
            ny = self.y + v * dt * math.sin(self.th)
        nth = (self.th + w * dt + math.pi) % (2 * math.pi) - math.pi
        if self.world.walls and self.world.collides(nx, ny, p.body_radius):
            # pushing into a wall: the body can still turn in place, but the wheels can't roll
            # forward, so the forward part of the wheel motion stalls (tyres grip, motors stall)
            self.stalled = True
            spin = (wr - wl) / 2.0
            self.left.phi += (-spin - wl) * dt
            self.right.phi += (spin - wr) * dt
            self.left.omega, self.right.omega = -spin, spin
        else:
            self.stalled = False
            self.distance += math.hypot(nx - self.x, ny - self.y)
            self.x, self.y = nx, ny
        self.th = nth
        # 2. bump switches
        self._update_bumps(t)
        # 3. IR sensors
        if t >= self._next_ir:
            self._update_ir()
            self._next_ir = t + IR_PERIOD
        # 4. new wheel speeds from the averaged driver inputs
        self.left.step(t, dt, p)
        self.right.step(t, dt, p)
        # 5. encoder edges for the coming interval
        k = p.encoder_cpr / (2 * math.pi)
        self.enc_l.schedule(t, self.left.phi * k, self.left.omega * k, dt)
        self.enc_r.schedule(t, self.right.phi * k, self.right.omega * k, dt)
        self._step_ev = self.m.sched.at(t + dt, self._step, owner="world")

    def _update_bumps(self, t):
        if not self.world.walls:
            return
        p = self.p
        new = 0
        for d, ang in self.world.closest_wall_points(self.x, self.y, p.body_radius + 0.004):
            rel = math.degrees((ang - self.th + math.pi) % (2 * math.pi) - math.pi)
            for i, (_, _, centre) in enumerate(pins.BUMP_SWITCHES):
                if abs(rel - centre) <= pins.BUMP_HALF_WIDTH_DEG:
                    new |= 1 << i
        if new != self.bumps:
            self.bumps = new
            for port in {pp for pp, _, _ in pins.BUMP_SWITCHES}:
                self.m.gpio.ext_changed(port, t)

    # ---------------------------------------------------------------- state
    def wheel_rpm(self):
        return (self.left.omega * 60 / (2 * math.pi), self.right.omega * 60 / (2 * math.pi))

    def leds(self):
        g = self.m.gpio
        out = {"LED1": g.is_output_high(*pins.LED_RED)}
        for k, pin in pins.LED_RGB.items():
            out["RGB_" + k] = g.is_output_high(*pin)
        for k, pin in pins.CHASSIS_LEDS.items():
            out[k] = g.is_output_high(*pin)
        return out
