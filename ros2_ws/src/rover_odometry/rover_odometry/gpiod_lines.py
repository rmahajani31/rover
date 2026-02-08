import gpiod

class _GpiodLine:
    """Small wrapper around a libgpiod line (same style as your motor code).""" 
    def __init__(self, chip: gpiod.Chip, offset: int):
        self.offset = offset
        self.line = chip.get_line(offset)

    def request_output(self, initial=0, consumer="rover"):
        self.line.request(
            consumer=consumer,
            type=gpiod.LINE_REQ_DIR_OUT,
            default_vals=[initial]
        )

    def request_input_events(self, consumer="rover_encoders"):
        """Request this GPIO line as input and enable BOTH edge events."""
        self.line.request(
            consumer=consumer,
            type=gpiod.LINE_REQ_EV_BOTH_EDGES
        )

    def set_value(self, v: int):
        self.line.set_value(int(v))

    def get_value(self) -> int:
        return int(self.line.get_value())

    def event_wait(self) -> bool: 
        """Wait for an edge event up to timeout_s seconds."""
        # sec = int(timeout_s)
        # nsec = int((timeout_s - sec) * 1e9)
        # return bool(self.line.event_wait(sec=sec, nsec=nsec))
        return bool(self.line.event_wait(sec=0, nsec=0))

    def event_read(self):
        """Consume one event (so it doesn’t pile up)."""
        return self.line.event_read()