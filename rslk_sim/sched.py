"""Discrete-event scheduler.  Time is a float in seconds of simulated time."""
import heapq
import itertools
import math


class Event:
    __slots__ = ("t", "fn", "cancelled", "owner")

    def __init__(self, t, fn, owner):
        self.t = t
        self.fn = fn
        self.cancelled = False
        self.owner = owner

    def cancel(self):
        self.cancelled = True


class Scheduler:
    def __init__(self):
        self._heap = []
        self._seq = itertools.count()

    def at(self, t, fn, owner="mcu"):
        """Call fn(t) at simulated time t.  Returns an Event that can be cancelled.

        `owner` groups events: an MCU reset cancels every "mcu" event but keeps
        "world" events (robot physics) running."""
        ev = Event(t, fn, owner)
        heapq.heappush(self._heap, (t, next(self._seq), ev))
        return ev

    def cancel_owner(self, owner):
        for _, _, ev in self._heap:
            if ev.owner == owner:
                ev.cancelled = True

    def next_time(self):
        heap = self._heap
        while heap and heap[0][2].cancelled:
            heapq.heappop(heap)
        return heap[0][0] if heap else math.inf

    def pop_due(self, t):
        """Pop the earliest event with time <= t, or return None."""
        heap = self._heap
        while heap:
            et, _, ev = heap[0]
            if ev.cancelled:
                heapq.heappop(heap)
                continue
            if et > t:
                return None
            heapq.heappop(heap)
            return ev
        return None

    def clear(self):
        self._heap.clear()
