# import gpiod
# from .gpiod_lines import _GpiodLine

# # Quadrature lookup:
# # state = (A<<1) | B  -> 00,01,11,10
# # index = (prev<<2) | new
# QUAD_TABLE = [
#     0,  +1, -1,  0,
#    -1,  0,  0, +1,
#    +1,  0,  0, -1,
#     0, -1, +1,  0
# ]

# class EncoderBackend:
#     """GPIO encoder backend using libgpiod, same 'chip + _GpiodLine' style."""

#     def __init__(self, gpiochip: str, left_a: int, left_b: int, right_a: int, right_b: int):
#         self.chip = gpiod.Chip(gpiochip)

#         # left wheel channels
#         self.la = _GpiodLine(self.chip, left_a)
#         self.lb = _GpiodLine(self.chip, left_b)

#         # right wheel channels
#         self.ra = _GpiodLine(self.chip, right_a)
#         self.rb = _GpiodLine(self.chip, right_b)

#         self.left_ticks = 0
#         self.right_ticks = 0

#         self._prev_left_state = 0
#         self._prev_right_state = 0

#     def setup(self):
#         """Request encoder lines as event inputs and initialize states."""
#         for ln in [self.la, self.lb, self.ra, self.rb]:
#             ln.request_input_events()

#         self._prev_left_state = (self.la.get_value() << 1) | self.lb.get_value()
#         self._prev_right_state = (self.ra.get_value() << 1) | self.rb.get_value()
    
#     def _update_left(self):
#         new_state = (self.la.get_value() << 1) | self.lb.get_value()
#         idx = (self._prev_left_state << 2) | new_state
#         self.left_ticks += QUAD_TABLE[idx]
#         self._prev_left_state = new_state

#     def _update_right(self):
#         new_state = (self.ra.get_value() << 1) | self.rb.get_value()
#         idx = (self._prev_right_state << 2) | new_state
#         self.right_ticks += QUAD_TABLE[idx]
#         self._prev_right_state = new_state

#     def poll_forever(self, sleep_s: float = 0.0002):
#         """
#         Call this in a thread. It drains queued events so you don't miss edges.
#         """
#         while True:
#             # Drain RIGHT events
#             while self.ra.event_wait():
#                 self.ra.event_read()
#                 self._update_right()
#             while self.rb.event_wait():
#                 self.rb.event_read()
#                 self._update_right()

#             # Drain LEFT events
#             while self.la.event_wait():
#                 self.la.event_read()
#                 self._update_left()
#             while self.lb.event_wait():
#                 self.lb.event_read()
#                 self._update_left()

# import gpiod
# import time
# from .gpiod_lines import _GpiodLine

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

#         # cached pin levels (IMPORTANT)
#         self._la_val = 0
#         self._lb_val = 0
#         self._ra_val = 0
#         self._rb_val = 0

#         self._prev_left_state = 0
#         self._prev_right_state = 0

#         # optional debugging
#         self.left_invalid = 0
#         self.right_invalid = 0

#     def setup(self):
#         for ln in [self.la, self.lb, self.ra, self.rb]:
#             ln.request_input_events()

#         # initialize cached values
#         self._la_val = self.la.get_value()
#         self._lb_val = self.lb.get_value()
#         self._ra_val = self.ra.get_value()
#         self._rb_val = self.rb.get_value()

#         self._prev_left_state = (self._la_val << 1) | self._lb_val
#         self._prev_right_state = (self._ra_val << 1) | self._rb_val

#     def _apply_left_edge(self, which: str, ev):
#         new_bit = 1 if ev.type == gpiod.LineEvent.RISING_EDGE else 0
#         if which == "a":
#             self._la_val = new_bit
#         else:
#             self._lb_val = new_bit

#         new_state = (self._la_val << 1) | self._lb_val
#         idx = (self._prev_left_state << 2) | new_state
#         delta = QUAD_TABLE[idx]
#         if delta == 0 and new_state != self._prev_left_state:
#             self.left_invalid += 1
#         self.left_ticks += delta
#         self._prev_left_state = new_state

#     def _apply_right_edge(self, which: str, ev):
#         new_bit = 1 if ev.type == gpiod.LineEvent.RISING_EDGE else 0
#         if which == "a":
#             self._ra_val = new_bit
#         else:
#             self._rb_val = new_bit

#         new_state = (self._ra_val << 1) | self._rb_val
#         idx = (self._prev_right_state << 2) | new_state
#         delta = QUAD_TABLE[idx]
#         if delta == 0 and new_state != self._prev_right_state:
#             self.right_invalid += 1
#         self.right_ticks += delta
#         self._prev_right_state = new_state

#     def poll_forever(self, sleep_s: float = 0.0002):
#         while True:

#             # RIGHT
#             while self.ra.event_wait():
#                 ev = self.ra.event_read()
#                 self._apply_right_edge("a", ev)
#             while self.rb.event_wait():
#                 ev = self.rb.event_read()
#                 self._apply_right_edge("b", ev)

#             # LEFT
#             while self.la.event_wait():
#                 ev = self.la.event_read()
#                 self._apply_left_edge("a", ev)
#             while self.lb.event_wait():
#                 ev = self.lb.event_read()
#                 self._apply_left_edge("b", ev)

import heapq
import time
import gpiod
from .gpiod_lines import _GpiodLine

# same QUAD_TABLE you already use
QUAD_TABLE = [
    0,  +1, -1,  0,
   -1,  0,  0, +1,
   +1,  0,  0, -1,
    0, -1, +1,  0
]

def _edge_is_rising(ev) -> bool:
    return ev.type == gpiod.LineEvent.RISING_EDGE

class EncoderBackend:
    def __init__(self, gpiochip: str, left_a: int, left_b: int, right_a: int, right_b: int):
        self.chip = gpiod.Chip(gpiochip)
        self.la = _GpiodLine(self.chip, left_a)
        self.lb = _GpiodLine(self.chip, left_b)
        self.ra = _GpiodLine(self.chip, right_a)
        self.rb = _GpiodLine(self.chip, right_b)

        self.left_ticks = 0
        self.right_ticks = 0

        # 2-bit states: (A<<1) | B
        self._left_state = 0
        self._right_state = 0

    def setup(self):
        for ln in [self.la, self.lb, self.ra, self.rb]:
            ln.request_input_events()

        # cache initial pin states
        self._la_val = self.la.get_value()
        self._lb_val = self.lb.get_value()
        self._ra_val = self.ra.get_value()
        self._rb_val = self.rb.get_value()

        self._prev_left_state = (self._la_val << 1) | self._lb_val
        self._prev_right_state = (self._ra_val << 1) | self._rb_val

    def _apply_left_edge(self, which: str, ev):
        new_bit = 1 if _edge_is_rising(ev) else 0
        if which == "a":
            self._la_val = new_bit
        else:
            self._lb_val = new_bit

        new_state = (self._la_val << 1) | self._lb_val
        idx = (self._prev_left_state << 2) | new_state
        self.left_ticks += QUAD_TABLE[idx]
        self._prev_left_state = new_state

    def _apply_right_edge(self, which: str, ev):
        new_bit = 1 if _edge_is_rising(ev) else 0
        if which == "a":
            self._ra_val = new_bit
        else:
            self._rb_val = new_bit

        new_state = (self._ra_val << 1) | self._rb_val
        idx = (self._prev_right_state << 2) | new_state
        self.right_ticks += QUAD_TABLE[idx]
        self._prev_right_state = new_state

    def poll_forever(self, sleep_s: float = 0.0002):
        lines = [
            (self.ra, "right", "a"),
            (self.rb, "right", "b"),
            (self.la, "left",  "a"),
            (self.lb, "left",  "b"),
        ]

        while True:
            # Round-robin: handle at most ONE event per line per loop
            for ln, wheel, ch in lines:
                if ln.event_wait():   # your event_wait() is non-blocking
                    ev = ln.event_read()
                    if wheel == "right":
                        self._apply_right_edge(ch, ev)
                    else:
                        self._apply_left_edge(ch, ev)
