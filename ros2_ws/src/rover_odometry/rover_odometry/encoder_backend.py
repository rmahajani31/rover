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
