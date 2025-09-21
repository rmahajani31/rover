#!/usr/bin/env python3
import asyncio, time, math
from evdev import InputDevice, ecodes, list_devices
from gpiozero import DigitalOutputDevice, PWMOutputDevice

# ===== Pins (your mapping) =====
AIN1, AIN2, PWMA = 5, 6, 12
BIN1, BIN2, PWMB = 23, 24, 13
STBY = 22

ain1 = DigitalOutputDevice(AIN1, initial_value=False)
ain2 = DigitalOutputDevice(AIN2, initial_value=False)
bin1 = DigitalOutputDevice(BIN1, initial_value=False)
bin2 = DigitalOutputDevice(BIN2, initial_value=False)
stby = DigitalOutputDevice(STBY, initial_value=False)
pwmA = PWMOutputDevice(PWMA, frequency=1000, initial_value=0.0)
pwmB = PWMOutputDevice(PWMB, frequency=1000, initial_value=0.0)

# ===== Drive tuning =====
INVERT_B       = True   # right side mirrored
TRIM_B         = 0.00
MAX_SPEED      = 0.85
SMOOTH         = 0.22   # smoothing on PWM magnitude
IDLE_TIMEOUT   = 0.8    # stop if no events for this long (s)

DEADZONE       = 0.06   # hard center deadzone per axis
MIN_PWM_DRIVE  = 0.18   # floor to beat static friction when driving
MIN_PWM_PIVOT  = 0.22   # floor when pivoting

# Zero-cross guard (prevents a brief kick the wrong way on direction flips)
ZERO_HOLD      = 0.10   # if current PWM > this, hold at 0 until it decays before flipping

# ===== Buttons =====
BTN_ESTOP  = {ecodes.BTN_EAST, ecodes.BTN_START}                       # B or START
BTN_SWAP   = {ecodes.BTN_SELECT, getattr(ecodes, "BTN_BACK", 316)}     # SELECT/BACK
BTN_INV_T  = {ecodes.BTN_TL}                                           # L1 toggles throttle sign
BTN_INV_S  = {ecodes.BTN_TR}                                           # R1 toggles steer sign

# ---------- helpers ----------
def clamp01(x): return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)
def clip11(x):  return -1.0 if x < -1.0 else (1.0 if x > 1.0 else x)

def set_motor_A(forward: bool):
    # Your “forward” = IN1 LOW, IN2 HIGH
    if forward: ain1.off(); ain2.on()
    else:       ain1.on();  ain2.off()

def set_motor_B(forward: bool):
    fwd = (not forward) if INVERT_B else forward
    if fwd: bin1.off(); bin2.on()
    else:   bin1.on();  bin2.off()

def set_speeds(vA: float, vB: float):
    pwmA.value = clamp01(vA)
    pwmB.value = clamp01(vB + TRIM_B)

def stop_now():
    pwmA.value = 0.0; pwmB.value = 0.0

def looks_like_gamepad(dev: InputDevice) -> bool:
    caps = dev.capabilities()
    has_abs = ecodes.EV_ABS in caps
    has_key = ecodes.EV_KEY in caps
    name = (dev.name or "").lower()
    branded = any(k in name for k in ["8bitdo","gamepad","controller","xbox","wireless","pro"])
    return has_abs and has_key and branded

def make_normalizer(absinfo, invert=False, deadzone=DEADZONE):
    mid  = (absinfo.max + absinfo.min) / 2.0
    span = (absinfo.max - absinfo.min) / 2.0 or 1.0
    flat = (absinfo.flat or 0) / span if span else 0.0
    dz = max(flat, deadzone)
    def f(v):
        x = (v - mid) / span
        if -dz < x < dz: return 0.0
        x = clip11(x)
        return -x if invert else x
    return f

def find_axis_pair(dev: InputDevice):
    """Return (vert_code, horz_code, invert_vert_default, invert_horz_default)
       Priority: (X,Y) -> (RX,RY) -> (HAT0X,HAT0Y)."""
    caps_abs_list = dev.capabilities().get(ecodes.EV_ABS, [])
    caps_abs = {code: dev.absinfo(code) for code, _ in caps_abs_list}
    def has(*codes): return all(c in caps_abs for c in codes)
    if has(ecodes.ABS_X, ecodes.ABS_Y):
        return (ecodes.ABS_Y, ecodes.ABS_X, True, False)      # Y inverted so UP=forward
    if has(ecodes.ABS_RX, ecodes.ABS_RY):
        return (ecodes.ABS_RY, ecodes.ABS_RX, True, False)
    if has(ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
        return (ecodes.ABS_HAT0Y, ecodes.ABS_HAT0X, False, False)  # digital fallback
    return None

def pick_working_device_and_axes():
    paths = sorted(list_devices())
    primary = [p for p in paths if looks_like_gamepad(InputDevice(p))]
    fallback = [p for p in paths if p not in primary]
    for p in primary + fallback:
        dev = InputDevice(p)
        caps = dev.capabilities()
        if ecodes.EV_ABS not in caps or ecodes.EV_KEY not in caps:
            continue
        ap = find_axis_pair(dev)
        if ap:
            v,h,inv_v,inv_h = ap
            return dev, v, h, inv_v, inv_h
    raise RuntimeError("No event device exposes a real stick pair (X/Y, RX/RY, or HAT0X/HAT0Y).")

def with_floor(u: float, floor: float) -> float:
    if abs(u) < 1e-6: return 0.0
    return math.copysign(floor + (1.0 - floor) * abs(u), u)

# ---------- main ----------
async def main():
    dev, THROTTLE_AXIS, STEER_AXIS, t_inv, s_inv = pick_working_device_and_axes()
    print(f"Using: {dev.path}  ({dev.name})")
    print(f"Axes: throttle={ecodes.ABS.get(THROTTLE_AXIS)} inv={t_inv}  "
          f"steer={ecodes.ABS.get(STEER_AXIS)} inv={s_inv}")
    stby.on()

    # Build scalers
    S_T = make_normalizer(dev.absinfo(THROTTLE_AXIS), invert=t_inv)
    S_S = make_normalizer(dev.absinfo(STEER_AXIS),    invert=s_inv)

    throttle = steer = 0.0
    curA = curB = 0.0
    last_event = time.time()

    # Track last *applied* motor directions for zero-cross guard
    last_dir_left  = +1
    last_dir_right = +1

    print("Controls: LEFT stick. SELECT swaps axes. L1/R1 invert throttle/steer. B/START = E-STOP.\n")

    try:
        async for e in dev.async_read_loop():
            last_event = time.time()

            # Buttons: swap/invert/estop
            if e.type == ecodes.EV_KEY and e.value == 1:
                c = e.code
                if c in BTN_ESTOP:
                    stop_now(); throttle = steer = curA = curB = 0.0
                    continue
                if c in BTN_SWAP:
                    THROTTLE_AXIS, STEER_AXIS = STEER_AXIS, THROTTLE_AXIS
                    S_T, S_S = S_S, S_T
                    print(f"SWAP → throttle={ecodes.ABS.get(THROTTLE_AXIS)}  steer={ecodes.ABS.get(STEER_AXIS)}")
                    stop_now(); throttle = steer = curA = curB = 0.0
                    continue
                if c in BTN_INV_T:
                    t_inv = not t_inv; S_T = make_normalizer(dev.absinfo(THROTTLE_AXIS), invert=t_inv)
                    print(f"Throttle invert -> {t_inv}")
                    continue
                if c in BTN_INV_S:
                    s_inv = not s_inv; S_S = make_normalizer(dev.absinfo(STEER_AXIS), invert=s_inv)
                    print(f"Steer invert -> {s_inv}")
                    continue

            # Axes
            if e.type == ecodes.EV_ABS:
                if e.code == THROTTLE_AXIS: throttle = S_T(e.value)
                elif e.code == STEER_AXIS:  steer    = S_S(e.value)

                # If fully centered, hard stop
                if abs(throttle) < DEADZONE and abs(steer) < DEADZONE:
                    stop_now(); curA = curB = 0.0
                    continue

                # === Arcade mixing (no sticky direction): throttle sign ALWAYS decides ===
                # Keep steering consistent in reverse (RIGHT always turns the nose right)
                dir_sign = 1.0 if throttle >= 0 else -1.0
                f = throttle                              # signed forward from stick
                steer_eff = steer * dir_sign              # flip steer sense in reverse

                l = f + steer_eff
                r = f - steer_eff

                # Normalize diagonals for a circular response
                scale = max(1.0, abs(f) + abs(steer))
                l = clip11(l / scale); r = clip11(r / scale)

                # Apply floors
                if abs(f) >= DEADZONE:  # driving
                    if l != 0: l = with_floor(l, MIN_PWM_DRIVE)
                    if r != 0: r = with_floor(r, MIN_PWM_DRIVE)
                else:                   # pivot (throttle near zero)
                    if l != 0: l = with_floor(l, MIN_PWM_PIVOT)
                    if r != 0: r = with_floor(r, MIN_PWM_PIVOT)

                # === Zero-cross guard per side ===
                desired_dir_left  = +1 if l >= 0 else -1
                desired_dir_right = +1 if r >= 0 else -1
                magL = abs(l); magR = abs(r)

                dir_out_left  = desired_dir_left
                dir_out_right = desired_dir_right
                tgtA = magL; tgtB = magR

                if desired_dir_left != last_dir_left and curA > ZERO_HOLD:
                    dir_out_left = last_dir_left
                    tgtA = 0.0
                if desired_dir_right != last_dir_right and curB > ZERO_HOLD:
                    dir_out_right = last_dir_right
                    tgtB = 0.0

                # Apply directions + smoothed PWM
                set_motor_A(dir_out_left  >= 0)
                set_motor_B(dir_out_right >= 0)
                curA = (1 - SMOOTH) * curA + SMOOTH * (tgtA * MAX_SPEED)
                curB = (1 - SMOOTH) * curB + SMOOTH * (tgtB * MAX_SPEED)
                set_speeds(curA, curB)

                last_dir_left  = dir_out_left
                last_dir_right = dir_out_right

            # Safety: stop if events stall
            if time.time() - last_event > IDLE_TIMEOUT:
                stop_now(); curA = curB = 0.0
    except KeyboardInterrupt:
        pass
    finally:
        stop_now()

if __name__ == "__main__":
    asyncio.run(main())