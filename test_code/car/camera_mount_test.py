import time
from math import cos, pi
from adafruit_servokit import ServoKit

# --- CONFIG ---
I2C_ADDRESS = 0x40
PAN_CH   = 4
TILT_CH  = 0

TILT_MIN, TILT_MAX = 40, 130

MIN_US = 550       # tighten/loosen to match your servos
MAX_US = 2450
ACTUATION_RANGE = 360

# Target motion characteristics
DEFAULT_SPEED_DPS  = 120.0   # max speed in degrees per second (tweak)
DEFAULT_RATE_HZ    = 150     # update rate; 120–200 is usually smooth
USE_EASING = True            # cosine S-curve easing

kit = ServoKit(channels=16, address=I2C_ADDRESS)
kit.frequency = 50

for ch in (PAN_CH, TILT_CH):
    s = kit.servo[ch]
    s.set_pulse_width_range(MIN_US, MAX_US)
    s.actuation_range = ACTUATION_RANGE

def clamp_tilt(a):
    try:
        lo, hi = TILT_MIN, TILT_MAX
        return max(lo, min(hi, float(a)))
    except NameError:
        return float(a)

def move_from_current_tilt(to_deg):
    cur = kit.servo[TILT_CH].angle
    if cur is None:
        # if you previously relaxed, re-enable by setting the current logical angle
        cur = 90.0
        kit.servo[TILT_CH].angle = cur
        time.sleep(0.2)
    move_smooth(
        TILT_CH,
        clamp_tilt(cur),
        clamp_tilt(to_deg),
        speed_dps=90, rate_hz=150, easing=True
    )

def set_angle(ch, angle):
    # allow floats; library accepts them
    kit.servo[ch].angle = max(0.0, min(float(ACTUATION_RANGE), float(angle)))

def move_smooth(ch, start_deg, end_deg, speed_dps=DEFAULT_SPEED_DPS, rate_hz=DEFAULT_RATE_HZ, easing=USE_EASING):
    """
    Smooth, time-based move from start_deg to end_deg at ~speed_dps.
    Uses cosine easing to ramp up/down by default.
    """
    start_deg = float(start_deg)
    end_deg   = float(end_deg)
    dist = abs(end_deg - start_deg)
    if dist == 0:
        return

    # duration based on desired speed (cap minimum so very small moves don't snap)
    duration = max(dist / float(speed_dps), 0.15)
    dt = 1.0 / float(rate_hz)
    steps = max(1, int(duration / dt))

    t0 = time.monotonic()
    for i in range(steps + 1):
        # normalized progress 0..1 based on time (not loop count) for stable cadence
        elapsed = time.monotonic() - t0
        u = min(1.0, elapsed / duration)

        if easing:
            # cosine ease-in-out (S-curve)
            e = 0.5 * (1.0 - cos(pi * u))
        else:
            e = u

        angle = start_deg + (end_deg - start_deg) * e
        set_angle(ch, angle)

        # sleep to maintain rate (adjust for code execution time)
        next_tick = t0 + (i + 1) * dt
        remaining = next_tick - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)

def go_home(pan=90, tilt=90):
    move_smooth(PAN_CH, kit.servo[PAN_CH].angle or 90, pan)
    move_smooth(TILT_CH, kit.servo[TILT_CH].angle or 90, tilt)

def relax(*chs):
    # stop sending pulses so the servo doesn’t hold torque
    for ch in chs:
        kit.servo[ch].angle = None

if __name__ == "__main__":
    # center
    # set_angle(PAN_CH, 90); set_angle(TILT_CH, 90); time.sleep(0.4)
    # move_smooth(TILT_CH, 60, 90, speed_dps=90, rate_hz=150, easing=True)
    # move_smooth(TILT_CH, 90, 60, speed_dps=90, rate_hz=150, easing=True)
    # move_from_current_tilt(60)
    # move_from_current_tilt(90)
    kit.servo[PAN_CH].angle = 180
    kit.servo[TILT_CH].angle = 180
    # time.sleep(0.5)
    # relax(PAN_CH, TILT_CH)

    # try:
    #     while True:
    #         # Smooth PAN sweep
    #         move_smooth(PAN_CH, 60, 150, speed_dps=120, rate_hz=150, easing=True)
    #         move_smooth(PAN_CH, 150, 60, speed_dps=120, rate_hz=150, easing=True)

    #         # Smooth TILT sweep (smaller range to avoid binds)
    #         move_smooth(TILT_CH, 60, 120, speed_dps=90, rate_hz=150, easing=True)
    #         move_smooth(TILT_CH, 120, 60, speed_dps=90, rate_hz=150, easing=True)
    # except KeyboardInterrupt:
    #     go_home()
    #     print("\nDone.")
