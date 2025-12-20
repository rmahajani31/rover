# tb6612_forward_reverse_turns_gpiozero_fixed_dirs.py
# Run with: sudo python3 tb6612_forward_reverse_turns_gpiozero_fixed_dirs.py
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep

# === Pin map (BCM) ===
AIN1, AIN2, PWMA = 5, 6, 12
BIN1, BIN2, PWMB = 16, 24, 13
STBY = 25

# === Devices ===
ain1 = DigitalOutputDevice(AIN1, initial_value=False)
ain2 = DigitalOutputDevice(AIN2, initial_value=False)
bin1 = DigitalOutputDevice(BIN1, initial_value=False)
bin2 = DigitalOutputDevice(BIN2, initial_value=False)
stby = DigitalOutputDevice(STBY, initial_value=False)

# Software PWM; you can try 20000 for quieter tone, but gpiozero is software PWM.
pwmA = PWMOutputDevice(PWMA, frequency=1000, initial_value=0.0)
pwmB = PWMOutputDevice(PWMB, frequency=1000, initial_value=0.0)

# === Tuning ===
# Assumption: Motor A = LEFT, Motor B = RIGHT.
# INVERT_B True if your right motor is mirrored so that "forward" makes the robot go forward.
INVERT_B = True
TRIM_B   = 0.00      # slow (-) or speed up (+) right motor, e.g., -0.05 for -5%

def clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))

def standby(enable: bool):
    stby.on() if enable else stby.off()

# --- Direction logic flipped so that "forward" = IN1 LOW, IN2 HIGH ---
def set_motor_A(forward: bool):
    if forward:
        ain1.off(); ain2.on()   # FORWARD (your chassis)
    else:
        ain1.on();  ain2.off()  # REVERSE

def set_motor_B(forward: bool):
    fwd = (not forward) if INVERT_B else forward
    if fwd:
        bin1.off(); bin2.on()   # FORWARD (your chassis)
    else:
        bin1.on();  bin2.off()  # REVERSE

def set_speed(v: float):
    """Legacy: same PWM to both sides (with trim on B)."""
    v = clamp01(v)
    pwmA.value = v
    pwmB.value = clamp01(v + TRIM_B)

def set_speeds(vA: float, vB: float):
    """Independent PWM per side (with trim on B/right)."""
    pwmA.value = clamp01(vA)
    pwmB.value = clamp01(vB + TRIM_B)

def coast():
    # High-Z by entering standby
    pwmA.value = 0.0; pwmB.value = 0.0
    standby(False)

def brake_short():
    # IN1=IN2=HIGH + PWM=100% => dynamic brake
    standby(True)
    ain1.on(); ain2.on()
    bin1.on(); bin2.on()
    pwmA.value = 1.0; pwmB.value = 1.0

def ramp_to(target, step=0.05, dt=0.03):
    """Legacy: ramp both sides together to 'target'."""
    cur = pwmA.value
    n = int(abs(target - cur) / step) or 1
    for i in range(1, n + 1):
        t = cur + (target - cur) * (i / n)
        set_speed(t); sleep(dt)

def ramp_to_pair(tA, tB, step=0.05, dt=0.03):
    """Ramp both channels (A,B) to (tA,tB) together."""
    cA, cB = pwmA.value, pwmB.value
    nA = int(abs(tA - cA) / step)
    nB = int(abs(tB - cB) / step)
    n = max(nA, nB, 1)
    for i in range(1, n + 1):
        vA = cA + (tA - cA) * (i / n)
        vB = cB + (tB - cB) * (i / n)
        set_speeds(vA, vB); sleep(dt)

def run_segment(dirA: bool, dirB: bool, vA: float, vB: float, seconds: float):
    """Lowest-level move: set directions, ramp up, hold, ramp down, brake, coast."""
    standby(True)
    set_motor_A(dirA)
    set_motor_B(dirB)
    ramp_to_pair(vA, vB)
    sleep(seconds)
    ramp_to_pair(0.0, 0.0)
    brake_short(); sleep(0.2)
    coast()

# === High-level motions ===

def drive(forward=True, seconds=1.5, speed=0.6):
    """Straight line forward/reverse."""
    run_segment(dirA=forward, dirB=forward, vA=speed, vB=speed, seconds=seconds)

def pivot_left(seconds=0.8, speed=0.6):
    """Turn in place, nose rotates left: left wheel reverse, right wheel forward."""
    run_segment(dirA=False, dirB=True, vA=speed, vB=speed, seconds=seconds)

def pivot_right(seconds=0.8, speed=0.6):
    """Turn in place, nose rotates right: left wheel forward, right wheel reverse."""
    run_segment(dirA=True, dirB=False, vA=speed, vB=speed, seconds=seconds)

def forward_left(seconds=1.0, speed=0.6, ratio=0.5):
    """Arc left while moving forward (skid steer): slow left side."""
    vL = clamp01(speed * ratio)
    vR = speed
    run_segment(dirA=True, dirB=True, vA=vL, vB=vR, seconds=seconds)

def forward_right(seconds=1.0, speed=0.6, ratio=0.5):
    """Arc right while moving forward: slow right side."""
    vL = speed
    vR = clamp01(speed * ratio)
    run_segment(dirA=True, dirB=True, vA=vL, vB=vR, seconds=seconds)

def reverse_left(seconds=1.0, speed=0.6, ratio=0.5):
    """Arc left while reversing: slow inner (left) side in reverse."""
    vL = clamp01(speed * ratio)
    vR = speed
    run_segment(dirA=False, dirB=False, vA=vL, vB=vR, seconds=seconds)

def reverse_right(seconds=1.0, speed=0.6, ratio=0.5):
    """Arc right while reversing: slow inner (right) side in reverse."""
    vL = speed
    vR = clamp01(speed * ratio)
    run_segment(dirA=False, dirB=False, vA=vL, vB=vR, seconds=seconds)

# === Demo ===
if __name__ == "__main__":
    try:
        print("Forward…")
        drive(forward=True,  seconds=1.5, speed=0.6); sleep(0.5)

        print("Reverse…")
        drive(forward=False, seconds=1.5, speed=0.6); sleep(0.5)

        print("Pivot left…")
        pivot_left(seconds=0.8, speed=0.6); sleep(0.5)

        print("Pivot right…")
        pivot_right(seconds=0.8, speed=0.6); sleep(0.5)

        print("Forward-left arc…")
        forward_left(seconds=1.0, speed=0.6, ratio=0.5); sleep(0.5)

        print("Forward-right arc…")
        forward_right(seconds=1.0, speed=0.6, ratio=0.5); sleep(0.5)

        print("Reverse-left arc…")
        reverse_left(seconds=1.0, speed=0.6, ratio=0.5); sleep(0.5)

        print("Reverse-right arc…")
        reverse_right(seconds=1.0, speed=0.6, ratio=0.5); sleep(0.5)

        print("Done.")
    finally:
        coast()