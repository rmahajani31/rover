import gpiod
from .gpiod_lines import _GpiodLine

# Quadrature lookup:
# state = (A<<1) | B  -> 00,01,11,10
# index = (prev<<2) | new
QUAD_TABLE = [
    0,  +1, -1,  0,
   -1,  0,  0, +1,
   +1,  0,  0, -1,
    0, -1, +1,  0
]

class EncoderBackend:
    """GPIO encoder backend using libgpiod, same 'chip + _GpiodLine' style."""

    def __init__(self, gpiochip: str, left_a: int, left_b: int, right_a: int, right_b: int):
        self.chip = gpiod.Chip(gpiochip)

        # left wheel channels
        self.la = _GpiodLine(self.chip, left_a)
        self.lb = _GpiodLine(self.chip, left_b)

        # right wheel channels
        self.ra = _GpiodLine(self.chip, right_a)
        self.rb = _GpiodLine(self.chip, right_b)

        self.left_ticks = 0
        self.right_ticks = 0

        self._prev_left_state = 0
        self._prev_right_state = 0

    def setup(self):
        """Request encoder lines as event inputs and initialize states."""
        for ln in [self.la, self.lb, self.ra, self.rb]:
            ln.request_input_events()

        self._prev_left_state = (self.la.get_value() << 1) | self.lb.get_value()
        self._prev_right_state = (self.ra.get_value() << 1) | self.rb.get_value()
    
    def _update_left(self):
        new_state = (self.la.get_value() << 1) | self.lb.get_value()
        idx = (self._prev_left_state << 2) | new_state
        self.left_ticks += QUAD_TABLE[idx]
        self._prev_left_state = new_state

    def _update_right(self):
        new_state = (self.ra.get_value() << 1) | self.rb.get_value()
        idx = (self._prev_right_state << 2) | new_state
        self.right_ticks += QUAD_TABLE[idx]
        self._prev_right_state = new_state

    def poll_forever(self, sleep_s: float = 0.0002):
        """
        Call this in a thread. It drains queued events so you don't miss edges.
        """
        while True:
            # Drain RIGHT events
            while self.ra.event_wait():
                self.ra.event_read()
                self._update_right()
            while self.rb.event_wait():
                self.rb.event_read()
                self._update_right()

            # Drain LEFT events
            while self.la.event_wait():
                self.la.event_read()
                self._update_left()
            while self.lb.event_wait():
                self.lb.event_read()
                self._update_left()