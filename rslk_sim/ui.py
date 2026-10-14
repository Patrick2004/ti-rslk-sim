"""pygame window: world view + robot + instrument panel."""
import math
import os
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402

from .robot import pins  # noqa: E402
from .session import Session  # noqa: E402

PANEL_W = 430
BG = (28, 30, 34)
PANEL_BG = (38, 41, 46)
FG = (225, 228, 232)
DIM = (140, 146, 155)
ACCENT = (90, 170, 255)
WARN = (255, 190, 80)
ERR = (255, 100, 90)
SPEEDS = [0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 10.0, math.inf]
TEXAS_WINDOWS = [0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5]    # seconds shown by the TExaS view
TRACE_COLORS = [(255, 210, 80), (120, 220, 120), (100, 190, 255), (255, 130, 200),
                (255, 150, 90), (180, 160, 255), (120, 230, 230)]
WATCH_ROWS = 10

HELP = [
    "Space pause/run    R restart program    H robot to start",
    "+ / - speed   T trail   W next world   1 / 2 SW1 / SW2",
    "Tab type into UART (Esc stops)   [ ] TExaS zoom",
    "Drag robot: move    Right-drag or wheel: turn",
    "Click a bump box to hold that switch pressed",
]


class Console:
    def __init__(self, max_lines=400):
        self.lines = [""]
        self.max_lines = max_lines
        self.cr = False

    def feed(self, byte):
        if byte == 10:
            self.lines.append("")
            self.cr = False
        elif byte == 13:
            self.cr = True
        else:
            if self.cr:
                self.lines.append("")
                self.cr = False
            ch = chr(byte) if 32 <= byte < 127 else ("    " if byte == 9 else "·")
            self.lines[-1] += ch
        if len(self.lines) > self.max_lines:
            del self.lines[: len(self.lines) - self.max_lines]


class App:
    def __init__(self, args, world_path, opts):
        pygame.init()
        pygame.display.set_caption(f"RSLK simulator - {os.path.basename(args.program)}")
        self.screen = pygame.display.set_mode((1360, 860), pygame.RESIZABLE)
        self.font = pygame.font.SysFont("consolas,dejavusansmono,couriernew", 15)
        self.small = pygame.font.SysFont("consolas,dejavusansmono,couriernew", 13)
        self.big = pygame.font.SysFont("segoeui,dejavusans,arial", 20, bold=True)
        self.args = args
        self.console = Console()
        self.log = lambda msg: print(msg, flush=True)
        self.session = Session(args.program, world_path, log=self.log, **opts)
        self.session.uart_listeners.append(lambda b, t: self.console.feed(b))
        self.session.m.trace_io = args.trace_io
        self.speed_i = min(range(len(SPEEDS)), key=lambda i: abs(SPEEDS[i] - args.speed))
        self.running = True
        self.typing = False
        self.typed = ""
        self.trail = []
        self.show_trail = True
        self.drag = None
        self.bump_rects = []
        self.sw_rects = {}
        self.held_keys = set()
        self.ratio = 0.0
        self.texas_win_i = TEXAS_WINDOWS.index(0.02)
        self.last_frame = time.perf_counter()
        self.stub = None
        if getattr(args, "gdb", None):
            from .gdbstub import GdbStub
            self.stub = GdbStub(self.session, args.gdb, log=self.log)
            self.running = False          # wait for the debugger (Space runs anyway)
        self.world_path = world_path
        self._prepare_floor()

    # ----------------------------------------------------------------- view
    def _prepare_floor(self):
        w = self.session.world
        sw, sh = self.screen.get_size()
        vw, vh = sw - PANEL_W - 20, sh - 20
        self.scale = min(vw / w.width, vh / w.height)       # pixels per metre
        fw, fh = int(w.width * self.scale), int(w.height * self.scale)
        self.view_origin = (10 + (vw - fw) // 2, 10 + (vh - fh) // 2)
        self.floor_img = pygame.transform.smoothscale(w.floor, (fw, fh))

    def to_screen(self, x, y):
        ox, oy = self.view_origin
        return (ox + x * self.scale, oy + (self.session.world.height - y) * self.scale)

    def to_world(self, px, py):
        ox, oy = self.view_origin
        return ((px - ox) / self.scale, self.session.world.height - (py - oy) / self.scale)

    # ----------------------------------------------------------------- loop
    def run(self):
        clock = pygame.time.Clock()
        shot = getattr(self.args, "screenshot", None)
        while True:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    pygame.quit()
                    return 0
                self.handle(ev)
            if self.stub:
                self.stub.poll()
                if self.stub.connected and not self.running:
                    self.running = True   # the debugger now controls run / pause
            if shot:
                # automated rendering check: simulate (recording the trail), save one frame, exit
                from .__main__ import parse_uart_args
                events = parse_uart_args(self.args.uart)
                m = self.session.m
                while m.t < self.args.screenshot_time and not m.halted:
                    while events and events[0][0] <= m.t:
                        m.uart_input(events.pop(0)[1])
                    m.run_until(min(m.t + 0.02, self.args.screenshot_time))
                    r = self.session.robot
                    self.trail.append((r.x, r.y))
                self.draw()
                pygame.image.save(self.screen, shot)
                pygame.quit()
                return 0
            self.step_sim()
            self.draw()
            pygame.display.flip()
            clock.tick(60)

    def step_sim(self):
        now = time.perf_counter()
        wall_dt = min(now - self.last_frame, 0.1)
        self.last_frame = now
        s = self.session
        m = s.m
        if not self.running or m.halted or (self.stub and self.stub.halted):
            self.ratio = 0.0
            return
        speed = SPEEDS[self.speed_i]
        t0 = m.t
        if speed == math.inf:
            m.run_until(m.t + 1.0, wall_budget=0.035)
        else:
            m.run_until(m.t + wall_dt * speed, wall_budget=0.035)
        if self.stub and m.debug_halt:
            self.stub.machine_stopped_at_breakpoint()
        if wall_dt > 0:
            self.ratio = 0.85 * self.ratio + 0.15 * ((m.t - t0) / wall_dt)
        r = s.robot
        if not self.trail or math.hypot(self.trail[-1][0] - r.x, self.trail[-1][1] - r.y) > 0.005:
            self.trail.append((r.x, r.y))
            if len(self.trail) > 4000:
                del self.trail[:1000]

    def restart(self):
        s = self.session
        s.reset(keep_pose=True)
        s.m.trace_io = self.args.trace_io
        self.console.feed(10)
        for ch in "--- program restarted ---\n":
            self.console.feed(ord(ch))
        self.running = True

    def cycle_world(self):
        """Switch to the next world in the worlds/ folder and restart the program there."""
        import glob
        from .__main__ import WORLDS_DIR
        worlds = sorted(glob.glob(os.path.join(WORLDS_DIR, "*.json")))
        if not worlds:
            return
        cur = os.path.abspath(self.world_path) if self.world_path else None
        idx = worlds.index(cur) + 1 if cur in worlds else 0
        self.world_path = worlds[idx % len(worlds)]
        from .robot.world import World
        s = self.session
        s.world_path = self.world_path
        s.world = World.load(self.world_path)
        s.reset()
        s.m.trace_io = self.args.trace_io
        self.trail.clear()
        self._prepare_floor()
        for ch in f"\n--- world: {s.world.name} ---\n":
            self.console.feed(ord(ch))

    # --------------------------------------------------------------- input
    def handle(self, ev):
        s = self.session
        if ev.type == pygame.VIDEORESIZE:
            self.screen = pygame.display.set_mode((max(ev.w, 900), max(ev.h, 600)), pygame.RESIZABLE)
            self._prepare_floor()
        elif ev.type == pygame.KEYDOWN:
            if self.typing:
                if ev.key == pygame.K_ESCAPE or ev.key == pygame.K_TAB:
                    self.typing = False
                elif ev.key == pygame.K_RETURN:
                    s.m.uart_input("\r")
                elif ev.key == pygame.K_BACKSPACE:
                    s.m.uart_input("\x08")
                elif ev.unicode:
                    s.m.uart_input(ev.unicode)
                return
            k = ev.key
            if k == pygame.K_SPACE:
                self.running = not self.running
            elif k == pygame.K_r:
                self.restart()
            elif k == pygame.K_h:
                s.robot.set_pose(*s.world.start)
                self.trail.clear()
            elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                self.speed_i = min(self.speed_i + 1, len(SPEEDS) - 1)
            elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                self.speed_i = max(self.speed_i - 1, 0)
            elif k == pygame.K_t:
                self.show_trail = not self.show_trail
                self.trail.clear()
            elif k == pygame.K_w:
                self.cycle_world()
            elif k == pygame.K_TAB:
                self.typing = True
            elif k == pygame.K_LEFTBRACKET:
                self.texas_win_i = max(self.texas_win_i - 1, 0)
            elif k == pygame.K_RIGHTBRACKET:
                self.texas_win_i = min(self.texas_win_i + 1, len(TEXAS_WINDOWS) - 1)
            elif k == pygame.K_1:
                s.robot.press_switch("SW1", True)
            elif k == pygame.K_2:
                s.robot.press_switch("SW2", True)
        elif ev.type == pygame.KEYUP:
            if ev.key == pygame.K_1:
                s.robot.press_switch("SW1", False)
            elif ev.key == pygame.K_2:
                s.robot.press_switch("SW2", False)
        elif ev.type == pygame.MOUSEBUTTONDOWN:
            pos = ev.pos
            for name, rect in self.sw_rects.items():
                if rect.collidepoint(pos) and ev.button == 1:
                    s.robot.press_switch(name, True)
                    self.drag = ("switch", name)
                    return
            for i, rect in enumerate(self.bump_rects):
                if rect.collidepoint(pos) and ev.button == 1:
                    s.robot.set_forced_bumps(s.robot.forced_bumps ^ (1 << i))
                    return
            r = s.robot
            rx, ry = self.to_screen(r.x, r.y)
            near = math.hypot(pos[0] - rx, pos[1] - ry) < max(18, r.p.body_radius * self.scale)
            if ev.button == 1 and near:
                self.drag = ("move", None)
            elif ev.button == 3 and near:
                self.drag = ("turn", None)
            elif ev.button in (4, 5) and near:
                r.set_pose(r.x, r.y, r.th + (0.1 if ev.button == 4 else -0.1))
        elif ev.type == pygame.MOUSEBUTTONUP:
            if self.drag and self.drag[0] == "switch":
                s.robot.press_switch(self.drag[1], False)
            self.drag = None
        elif ev.type == pygame.MOUSEMOTION and self.drag:
            r = s.robot
            if self.drag[0] == "move":
                x, y = self.to_world(*ev.pos)
                r.set_pose(min(max(x, 0.0), s.world.width), min(max(y, 0.0), s.world.height), r.th)
                self.trail.clear()
            elif self.drag[0] == "turn":
                x, y = self.to_world(*ev.pos)
                r.set_pose(r.x, r.y, math.atan2(y - r.y, x - r.x))

    # ---------------------------------------------------------------- draw
    def text(self, s, pos, color=FG, font=None):
        surf = (font or self.font).render(s, True, color)
        self.screen.blit(surf, pos)
        return surf.get_width()

    def draw(self):
        scr = self.screen
        scr.fill(BG)
        self.draw_world()
        self.draw_panel()

    def draw_world(self):
        scr = self.screen
        s = self.session
        w = s.world
        r = s.robot
        scr.blit(self.floor_img, self.view_origin)
        for x1, y1, x2, y2 in w.walls:
            pygame.draw.line(scr, (120, 75, 40), self.to_screen(x1, y1), self.to_screen(x2, y2), max(3, int(0.012 * self.scale)))
        if self.show_trail and len(self.trail) > 1:
            pygame.draw.lines(scr, (60, 140, 255), False, [self.to_screen(x, y) for x, y in self.trail], 2)
        c, sn = math.cos(r.th), math.sin(r.th)

        def body(fx, fy):
            return self.to_screen(r.x + c * fx - sn * fy, r.y + sn * fx + c * fy)

        # IR rays (only meaningful when there are walls)
        for name, (_pin, mx, my, ang) in (pins.IR_SENSORS.items() if w.walls else ()):
            d = r.ir_dist.get(name, 1.5)
            a = math.radians(ang)
            start = body(mx, my)
            end = body(mx + math.cos(a) * min(d, 0.8), my + math.sin(a) * min(d, 0.8))
            col = (255, 80, 80) if d < 0.8 else (255, 170, 170)
            pygame.draw.line(scr, col, start, end, 1)
            if d < 0.8:
                pygame.draw.circle(scr, (255, 60, 60), (int(end[0]), int(end[1])), 3)
        rad = r.p.body_radius * self.scale
        center = self.to_screen(r.x, r.y)
        surf = pygame.Surface((int(rad * 2 + 4), int(rad * 2 + 4)), pygame.SRCALPHA)
        pygame.draw.circle(surf, (40, 120, 70, 170), (int(rad + 2), int(rad + 2)), int(rad))
        scr.blit(surf, (center[0] - rad - 2, center[1] - rad - 2))
        pygame.draw.circle(scr, (20, 70, 40), (int(center[0]), int(center[1])), int(rad), 2)
        # bumper arcs
        pressed = r.bumps | r.forced_bumps
        for i, (_, _, ang) in enumerate(pins.BUMP_SWITCHES):
            a0 = math.radians(ang - pins.BUMP_HALF_WIDTH_DEG + 2)
            a1 = math.radians(ang + pins.BUMP_HALF_WIDTH_DEG - 2)
            pts = [body(math.cos(a) * (r.p.body_radius + 0.004), math.sin(a) * (r.p.body_radius + 0.004))
                   for a in [a0 + (a1 - a0) * k / 6 for k in range(7)]]
            pygame.draw.lines(scr, (255, 60, 60) if pressed >> i & 1 else (60, 60, 60), False, pts, 4)
        # wheels
        for side in (1, -1):
            wy = side * r.p.track / 2
            pygame.draw.line(scr, (25, 25, 25), body(-0.035, wy), body(0.035, wy), max(4, int(0.012 * self.scale)))
        pygame.draw.line(scr, (250, 250, 250), center, body(0.06, 0), 2)
        # line sensors
        g = s.m.gpio
        for bit in range(8):
            px, py = body(r.p.sensor_bar_x, pins.REFLECT_Y[bit])
            v = r.floor_under_sensor(bit)
            col = (int(255 * v), int(220 * v), 40)
            pygame.draw.circle(scr, col, (int(px), int(py)), max(3, int(0.004 * self.scale)))
        # chassis LEDs
        leds = r.leds()
        for name, (fx, fy, col) in {"front_left": (0.05, 0.05, (255, 210, 0)), "front_right": (0.05, -0.05, (255, 210, 0)),
                                     "back_left": (-0.06, 0.05, (255, 40, 40)), "back_right": (-0.06, -0.05, (255, 40, 40))}.items():
            if leds.get(name):
                p = body(fx, fy)
                pygame.draw.circle(scr, col, (int(p[0]), int(p[1])), 5)
        if s.program_changed():
            self.banner("The .out file was rebuilt - press R to load it", WARN)
        if s.m.halted:
            self.banner(s.m.halted, ERR)
        elif s.m.finished:
            self.banner("main() returned - the CPU idles in exit(); motors keep their last setting", WARN)

    def banner(self, msg, color):
        scr = self.screen
        vw = scr.get_width() - PANEL_W
        words, lines, cur = msg.split(), [], ""
        for wd in words:
            if self.font.size(cur + " " + wd)[0] > vw - 60:
                lines.append(cur)
                cur = wd
            else:
                cur = (cur + " " + wd).strip()
        lines.append(cur)
        h = 12 + 20 * len(lines)
        rect = pygame.Rect(20, 20, vw - 40, h)
        pygame.draw.rect(scr, (20, 20, 20), rect, border_radius=6)
        pygame.draw.rect(scr, color, rect, 2, border_radius=6)
        for i, ln in enumerate(lines):
            self.text(ln, (32, 26 + 20 * i), color)

    def draw_panel(self):
        scr = self.screen
        s = self.session
        m = s.m
        r = s.robot
        x0 = scr.get_width() - PANEL_W
        pygame.draw.rect(scr, PANEL_BG, (x0, 0, PANEL_W, scr.get_height()))
        x = x0 + 14
        y = 10
        self.text(os.path.basename(s.elf_path), (x, y), FG, self.big)
        y += 28
        self.text(f"world: {s.world.name}", (x, y), DIM, self.small)
        y += 20
        speed = SPEEDS[self.speed_i]
        debug = self.stub and self.stub.halted
        state = "STOPPED" if m.halted else ("DEBUGGER" if debug else "PAUSED" if not self.running
                                             else "FINISHED" if m.finished else "RUNNING")
        col = ERR if m.halted else (WARN if debug or not self.running else ACCENT)
        self.text(state, (x, y), col)
        sp = "max" if speed == math.inf else f"{speed:g}x"
        self.text(f"t = {m.t:8.3f} s   speed {sp} (actual {self.ratio:.2f}x)", (x + 80, y))
        y += 20
        self.text(f"MCLK {m.clk['MCLK'] / 1e6:g} MHz   SMCLK {m.clk['SMCLK'] / 1e6:g} MHz", (x, y), DIM, self.small)
        y += 18
        pc = m.describe_pc(m.pc)
        self.text(("in " + pc)[:60], (x, y), DIM, self.small)
        y += 24

        # LaunchPad
        leds = r.leds()
        self.text("LaunchPad", (x, y), ACCENT)
        lx = x + 100
        pygame.draw.circle(scr, (255, 40, 40) if leds["LED1"] else (70, 30, 30), (lx, y + 9), 8)
        self.text("LED1", (lx + 12, y + 1), DIM, self.small)
        rgb = (255 if leds["RGB_red"] else 40, 255 if leds["RGB_green"] else 40, 255 if leds["RGB_blue"] else 40)
        on = leds["RGB_red"] or leds["RGB_green"] or leds["RGB_blue"]
        pygame.draw.circle(scr, rgb if on else (50, 50, 50), (lx + 70, y + 9), 8)
        self.text("RGB", (lx + 82, y + 1), DIM, self.small)
        self.sw_rects = {}
        for i, name in enumerate(("SW1", "SW2")):
            rect = pygame.Rect(lx + 130 + i * 60, y, 52, 20)
            pygame.draw.rect(scr, ACCENT if r.switches[name] else (70, 75, 82), rect, border_radius=4)
            self.text(name, (rect.x + 10, rect.y + 2), FG, self.small)
            self.sw_rects[name] = rect
        y += 30

        # motors
        self.text("Motors", (x, y), ACCENT)
        rpm_l, rpm_r = r.wheel_rpm()
        for i, (label, mot, rpm) in enumerate((("L", r.left, rpm_l), ("R", r.right, rpm_r))):
            yy = y + 22 + i * 22
            self.text(label, (x, yy), FG, self.small)
            bar = pygame.Rect(x + 20, yy + 2, 180, 14)
            pygame.draw.rect(scr, (60, 64, 70), bar)
            mid = bar.x + bar.w // 2
            d = max(-1.0, min(1.0, mot.avg_drive))
            col = (90, 200, 120) if d >= 0 else (230, 120, 80)
            if d >= 0:
                pygame.draw.rect(scr, col, (mid, bar.y, int(d * bar.w / 2), bar.h))
            else:
                pygame.draw.rect(scr, col, (mid + int(d * bar.w / 2), bar.y, int(-d * bar.w / 2), bar.h))
            pygame.draw.line(scr, DIM, (mid, bar.y - 2), (mid, bar.y + bar.h + 2))
            sleep = " (asleep)" if mot.asleep else ""
            self.text(f"{d * 100:+5.0f}%  {rpm:+6.1f} rpm{sleep}", (x + 210, yy), FG, self.small)
        y += 72
        self.text(f"odometer {r.distance:6.2f} m" + ("   pushing a wall" if r.stalled else ""), (x, y), DIM, self.small)
        y += 24

        # bump switches
        self.text("Bump", (x, y), ACCENT)
        self.bump_rects = []
        pressed = r.bumps | r.forced_bumps
        for i in range(6):
            # draw left-to-right as seen from above: Bump5 (left) ... Bump0 (right)
            j = 5 - i
            rect = pygame.Rect(x + 60 + i * 44, y, 38, 20)
            colr = (230, 70, 60) if pressed >> j & 1 else (70, 75, 82)
            pygame.draw.rect(scr, colr, rect, border_radius=4)
            if r.forced_bumps >> j & 1:
                pygame.draw.rect(scr, WARN, rect, 2, border_radius=4)
            self.text(f"B{j}", (rect.x + 10, rect.y + 2), FG, self.small)
            self.bump_rects.append(None)
        self.bump_rects = [pygame.Rect(x + 60 + (5 - j) * 44, y, 38, 20) for j in range(6)]
        y += 28

        # reflectance
        self.text("Line", (x, y), ACCENT)
        for i in range(8):
            bit = 7 - i                     # leftmost sensor first
            v = r.floor_under_sensor(bit)
            rect = pygame.Rect(x + 60 + i * 33, y, 28, 20)
            g = int(v * 255)
            pygame.draw.rect(scr, (g, g, g), rect, border_radius=3)
            pygame.draw.rect(scr, (90, 95, 100), rect, 1, border_radius=3)
        self.text("floor under sensors 7..0", (x + 60, y + 22), DIM, self.small)
        y += 44

        # IR
        self.text("IR", (x, y), ACCENT)
        xx = x + 60
        for name in ("left", "center", "right"):
            d = r.ir_dist[name]
            ds = f"{d * 100:4.0f}cm" if d < 0.8 else " >80cm"
            self.text(f"{name[0].upper()} {ds} {r.ir_volts[name]:.2f}V", (xx, y + 1), FG, self.small)
            xx += 120
        y += 26

        # LCD
        if s.lcd.used:
            self.text("Nokia 5110", (x, y), ACCENT)
            y += 20
            px = 3
            rect = pygame.Rect(x, y, 84 * px + 8, 48 * px + 8)
            pygame.draw.rect(scr, (150, 175, 140), rect, border_radius=4)
            for yy in range(48):
                for xx in range(84):
                    if s.lcd.pixel(xx, yy):
                        pygame.draw.rect(scr, (25, 35, 25), (x + 4 + xx * px, y + 4 + yy * px, px, px))
            y += rect.h + 8

        # variables (like the CCS Expressions window)
        if s.watch.items:
            self.text("Variables", (x, y), ACCENT)
            self.text("globals in your lab file (more: --watch)", (x + 100, y + 1), DIM, self.small)
            y += 20
            items = s.watch.snapshot(m)
            shown = items[:WATCH_ROWS]
            width = min(max(len(label) for label, _ in shown), 26)
            for label, value in shown:
                self.text(f"{label[:width]:<{width}}  {value}"[:56], (x + 6, y), FG, self.small)
                y += 16
            if len(items) > len(shown):
                self.text(f"... and {len(items) - len(shown)} more", (x + 6, y), DIM, self.small)
                y += 16
            y += 8

        # TExaS logic analyzer / scope
        if s.texas.mode and s.texas.samples:
            y = self.draw_texas(x, y)

        # UART console
        self.text("UART (EUSCI_A0, 115200)", (x, y), ACCENT)
        if self.typing:
            self.text("typing - Esc to stop", (x + 240, y), WARN, self.small)
        else:
            self.text("Tab to type", (x + 300, y), DIM, self.small)
        y += 20
        help_h = 18 * len(HELP) + 10
        msg_lines = [t for _, t in m.messages[-3:]]
        msg_h = 16 * len(msg_lines) + (8 if msg_lines else 0)
        box_h = scr.get_height() - y - help_h - msg_h - 10
        rect = pygame.Rect(x, y, PANEL_W - 28, max(60, box_h))
        pygame.draw.rect(scr, (18, 19, 22), rect, border_radius=4)
        if self.typing:
            pygame.draw.rect(scr, WARN, rect, 1, border_radius=4)
        cols = (rect.w - 10) // self.small.size("M")[0]
        wrapped = []
        for line in self.console.lines[-200:]:
            while len(line) > cols:
                wrapped.append(line[:cols])
                line = line[cols:]
            wrapped.append(line)
        n = (rect.h - 8) // 16
        for i, line in enumerate(wrapped[-n:]):
            self.text(line, (rect.x + 5, rect.y + 4 + i * 16), (200, 230, 200), self.small)
        y = rect.bottom + 6
        for t in msg_lines:
            col = ERR if "STOPPED" in t else (WARN if "warning" in t else DIM)
            self.text(t[:70], (x, y), col, self.small)
            y += 16
        y = scr.get_height() - help_h
        for line in HELP:
            self.text(line, (x0 + 8, y), DIM, self.small)
            y += 18


    def draw_texas(self, x, y):
        """TExaSdisplay-style traces of the most recent samples (newest at the right)."""
        scr = self.screen
        tx = self.session.texas
        win = TEXAS_WINDOWS[self.texas_win_i]
        self.text(tx.title(), (x, y), ACCENT)
        self.text(f"{win * 1000:g} ms window   [ ] zoom", (x + 250, y + 1), DIM, self.small)
        y += 20
        label_w = 46
        width = PANEL_W - 28 - label_w
        x1 = x + label_w
        samples = tx.samples
        t_end = samples[-1][0]
        t_start = t_end - win
        recent = []
        for t, v in reversed(samples):
            if t < t_start:
                break
            recent.append((t, v))
        recent.reverse()
        k = width / win

        def px(t):
            return x1 + int((t - t_start) * k)

        if tx.mode == "scope":
            h = 80
            pygame.draw.rect(scr, (18, 19, 22), (x1, y, width, h))
            pts = [(px(t), y + h - 1 - int(v * (h - 2) / 255)) for t, v in recent]
            if len(pts) > 1:
                pygame.draw.lines(scr, TRACE_COLORS[0], False, pts, 1)
            self.text("3.3V", (x, y), DIM, self.small)
            self.text("0V", (x, y + h - 14), DIM, self.small)
            return y + h + 10
        row = 15
        n = len(tx.labels)
        pygame.draw.rect(scr, (18, 19, 22), (x1, y, width, row * n + 2))
        # only the samples where some channel changes matter for drawing
        changes = []
        prev = None
        for t, v in recent:
            if v != prev:
                changes.append((px(t), v))
                prev = v
        xr = x1 + width - 1
        for bit in range(n):
            # top row = highest channel, like TExaSdisplay
            yy = y + 1 + (n - 1 - bit) * row
            hi, lo = yy + 2, yy + row - 3
            col = TRACE_COLORS[bit % len(TRACE_COLORS)]
            self.text(tx.labels[bit], (x, yy), col, self.small)
            pts = []
            level = None
            for cx, v in changes:
                b = v >> bit & 1
                if b != level:
                    yv = hi if b else lo
                    if pts:
                        pts.append((cx, pts[-1][1]))
                    pts.append((cx, yv))
                    level = b
            if pts:
                pts.append((xr, pts[-1][1]))
                if len(pts) > 1:
                    pygame.draw.lines(scr, col, False, pts, 1)
        return y + row * n + 12


def run_ui(args, world_path, opts):
    app = App(args, world_path, opts)
    return app.run()
