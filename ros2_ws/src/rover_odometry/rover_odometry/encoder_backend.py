# import time
# import gpiod
# from .gpiod_lines import _GpiodLine

# # same QUAD_TABLE you already use
# QUAD_TABLE = [
#     0,  +1, -1,  0,
#    -1,  0,  0, +1,
#    +1,  0,  0, -1,
#     0, -1, +1,  0
# ]

# class EncoderBackend:
#     def __init__(self, gpiochip: str, left_a: int, left_b: int, right_a: int, right_b: int):
#         self.chip = gpiod.Chip(gpiochip)
#         self.la = _GpiodLine(self.chip, left_a)
#         self.lb = _GpiodLine(self.chip, left_b)
#         self.ra = _GpiodLine(self.chip, right_a)
#         self.rb = _GpiodLine(self.chip, right_b)

#         self.left_ticks = 0
#         self.right_ticks = 0

#         # prev 2-bit states: (A<<1) | B
#         self._prev_left_state = 0
#         self._prev_right_state = 0

#     def setup(self):
#         for ln in (self.la, self.lb, self.ra, self.rb):
#             ln.request_input_events()

#         # cache initial pin states (REAL levels)
#         la = self.la.get_value()
#         lb = self.lb.get_value()
#         ra = self.ra.get_value()
#         rb = self.rb.get_value()

#         self._prev_left_state = (la << 1) | lb
#         self._prev_right_state = (ra << 1) | rb

#     def _drain_events(self, ln) -> bool:
#         """
#         Drain all queued events for a line.
#         Returns True if we drained at least one event.
#         """
#         got_any = False
#         while ln.event_wait():          # your event_wait() is non-blocking
#             _ = ln.event_read()         # discard payload; we will sample pins instead
#             got_any = True
#         return got_any

#     def _step_left_from_pins(self):
#         la = self.la.get_value()
#         lb = self.lb.get_value()
#         new_state = (la << 1) | lb

#         idx = (self._prev_left_state << 2) | new_state
#         delta = QUAD_TABLE[idx]

#         if delta == 0 and new_state != self._prev_left_state:
#             print(f"Invalid state transition detected for left encoder: "
#                   f"{self._prev_left_state:02b}->{new_state:02b} (idx={idx})")
#         else:
#             # Only print nonzero deltas (optional)
#             if delta != 0:
#                 print(f"Left encoder: {delta}")

#         self.left_ticks += delta
#         self._prev_left_state = new_state

#     def _step_right_from_pins(self):
#         ra = self.ra.get_value()
#         rb = self.rb.get_value()
#         new_state = (ra << 1) | rb

#         idx = (self._prev_right_state << 2) | new_state
#         delta = QUAD_TABLE[idx]

#         if delta == 0 and new_state != self._prev_right_state:
#             print(f"Invalid state transition detected for right encoder: "
#                   f"{self._prev_right_state:02b}->{new_state:02b} (idx={idx})")
#         else:
#             if delta != 0:
#                 print(f"Right encoder: {delta}")

#         self.right_ticks += delta
#         self._prev_right_state = new_state

#     def poll_forever(self, sleep_s: float = 0.0002):
#         """
#         - Drain events so we never get backed up.
#         - Update using *actual pin levels* so stale events can't corrupt state.
#         """
#         while True:
#             # Drain + update LEFT if either channel has events
#             left_had = self._drain_events(self.la) | self._drain_events(self.lb)
#             if left_had:
#                 self._step_left_from_pins()

#             # Drain + update RIGHT if either channel has events
#             right_had = self._drain_events(self.ra) | self._drain_events(self.rb)
#             if right_had:
#                 self._step_right_from_pins()

import heapq
import time
import gpiod
from .gpiod_lines import _GpiodLine

QUAD_TABLE = [
    0,  +1, -1,  0,
   -1,  0,  0, +1,
   +1,  0,  0, -1,
    0, -1, +1,  0
]

def _edge_is_rising(ev) -> bool:
    return ev.type == gpiod.LineEvent.RISING_EDGE

def _event_ts_ns(ev) -> int:
    """
    Get a monotonic timestamp for ordering.
    Adjust if your wrapper exposes a different attribute.
    """
    # Common in libgpiod wrappers: ev.timestamp (ns) or ev.timestamp_ns
    for attr in ("timestamp_ns", "timestamp", "ts_ns"):
        if hasattr(ev, attr):
            v = getattr(ev, attr)
            return int(v() if callable(v) else v)
    return time.monotonic_ns()

class EncoderBackend:
    def __init__(self, gpiochip: str, left_a: int, left_b: int, right_a: int, right_b: int):
        self.chip = gpiod.Chip(gpiochip)
        self.la = _GpiodLine(self.chip, left_a)
        self.lb = _GpiodLine(self.chip, left_b)
        self.ra = _GpiodLine(self.chip, right_a)
        self.rb = _GpiodLine(self.chip, right_b)

        self.left_ticks = 0
        self.right_ticks = 0

        # cached pin levels for per-event updates
        self._la_val = 0
        self._lb_val = 0
        self._ra_val = 0
        self._rb_val = 0

        self._prev_left_state = 0
        self._prev_right_state = 0

        # debug counters
        self.left_invalid = 0
        self.right_invalid = 0
        self.ev_counts = {"la":0, "lb":0, "ra":0, "rb":0}

        self._heap = []  # (ts_ns, wheel, ch, rising)

    def setup(self):
        for ln in (self.la, self.lb, self.ra, self.rb):
            ln.request_input_events()  # MUST be BOTH edges in your wrapper

        # seed cached values from real pin reads
        self._la_val = self.la.get_value()
        self._lb_val = self.lb.get_value()
        self._ra_val = self.ra.get_value()
        self._rb_val = self.rb.get_value()

        self._prev_left_state  = (self._la_val << 1) | self._lb_val
        self._prev_right_state = (self._ra_val << 1) | self._rb_val

    def _drain_to_heap(self, ln, key: str, wheel: str, ch: str):
        while ln.event_wait():  # non-blocking
            ev = ln.event_read()
            self.ev_counts[key] += 1
            ts = _event_ts_ns(ev)
            rising = _edge_is_rising(ev)
            heapq.heappush(self._heap, (ts, wheel, ch, rising))

    def _apply_edge(self, wheel: str, ch: str, rising: bool):
        bit = 1 if rising else 0

        if wheel == "left":
            if ch == "a":
                self._la_val = bit
            else:
                self._lb_val = bit

            new_state = (self._la_val << 1) | self._lb_val
            idx = (self._prev_left_state << 2) | new_state
            delta = QUAD_TABLE[idx]

            if delta == 0:
                # key: DON'T advance prev state on invalid jump
                if new_state != self._prev_left_state:
                    self.left_invalid += 1
                return

            self.left_ticks += delta
            self._prev_left_state = new_state

        else:  # right
            if ch == "a":
                self._ra_val = bit
            else:
                self._rb_val = bit

            new_state = (self._ra_val << 1) | self._rb_val
            idx = (self._prev_right_state << 2) | new_state
            delta = QUAD_TABLE[idx]

            if delta == 0:
                if new_state != self._prev_right_state:
                    self.right_invalid += 1
                return

            self.right_ticks += delta
            self._prev_right_state = new_state

    def poll_forever(self, debug_every_s: float = 1.0):
        """
        Drain all 4 lines, process per-edge in timestamp order.
        Avoid sleeps and per-edge prints (both cause drops).
        """
        last_dbg = time.monotonic()

        while True:
            # 1) Drain everything available into a global heap
            self._drain_to_heap(self.la, "la", "left",  "a")
            self._drain_to_heap(self.lb, "lb", "left",  "b")
            self._drain_to_heap(self.ra, "ra", "right", "a")
            self._drain_to_heap(self.rb, "rb", "right", "b")

            # 2) Apply edges in chronological order
            while self._heap:
                _ts, wheel, ch, rising = heapq.heappop(self._heap)
                self._apply_edge(wheel, ch, rising)

            # 3) Occasional debug (cheap)
            now = time.monotonic()
            if debug_every_s and (now - last_dbg) >= debug_every_s:
                print(
                    "events/s:", self.ev_counts,
                    "invalid/s:", {"L": self.left_invalid, "R": self.right_invalid},
                    "ticks:", {"L": self.left_ticks, "R": self.right_ticks},
                )
                self.ev_counts = {k: 0 for k in self.ev_counts}
                self.left_invalid = 0
                self.right_invalid = 0
                last_dbg = now

