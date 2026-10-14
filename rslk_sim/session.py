"""A simulation session: program + machine + robot + world, rebuildable on reset."""
import os

from .elfload import Program
from .machine import Machine
from .robot import pins
from .robot.lcd import Nokia5110
from .robot.robot import Robot
from .robot.world import World
from .texas import TexasMonitor
from .watch import Watch


class Session:
    def __init__(self, elf_path, world_path=None, cpi=2.0, hle=True, idle_skip=True, log=print,
                 watch=(), texas_keep_all=False):
        self.elf_path = elf_path
        self.watch_names = list(watch)         # extra variables for the watch list
        self.texas_keep_all = texas_keep_all
        self.world_path = world_path
        self.cpi, self.hle, self.idle_skip = cpi, hle, idle_skip
        self.log = log
        self.world = World.load(world_path) if world_path else World.empty()
        self.uart_listeners = []          # extra fn(byte, t) callbacks, kept across resets
        self.rebuild_listeners = []       # fn() after the Machine is rebuilt (reset)
        self._mtime = None
        self.build()

    def build(self, keep_pose=None):
        self._mtime = os.path.getmtime(self.elf_path)
        self.prog = Program(self.elf_path)
        self.m = Machine(self.prog, cpi=self.cpi, hle=self.hle, idle_skip=self.idle_skip, log=self.log)
        self.robot = Robot(self.m, self.world)
        if keep_pose:
            self.robot.set_pose(*keep_pose)
        spi = next(u for u in self.m.uarts if u.name == pins.LCD["spi"])
        self.lcd = Nokia5110(self.m, spi, pins.LCD["dc"], pins.LCD["rst"])
        self.uart_text = bytearray()
        self.texas = TexasMonitor(self.m, keep_all=self.texas_keep_all)
        self.watch = Watch(self.prog, self.watch_names)
        self.m.uart_listeners.append(self._on_uart)

    def _on_uart(self, byte, t):
        if self.texas.consume(byte, t):
            return                         # a TExaS logic analyzer / scope sample, not text
        self.uart_text.append(byte)
        if len(self.uart_text) > 200_000:
            del self.uart_text[:100_000]
        for fn in self.uart_listeners:
            fn(byte, t)

    def program_changed(self):
        try:
            return os.path.getmtime(self.elf_path) != self._mtime
        except OSError:
            return False

    def reset(self, keep_pose=False):
        """Restart the program (reloading the .out file if it was rebuilt)."""
        pose = (self.robot.x, self.robot.y, self.robot.th) if keep_pose else None
        self.build(keep_pose=pose)
        for fn in self.rebuild_listeners:
            fn()

    def run_for(self, seconds, wall_budget=None):
        self.m.run_until(self.m.t + seconds, wall_budget)
