# tb6612_forward_reverse_gpiozero_fixed_dirs.py
# Run with: sudo python3 tb6612_forward_reverse_gpiozero_fixed_dirs.py
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep

# === Pin map (BCM) ===
AIN1, AIN2, PWMA = 5, 6, 12
BIN1, BIN2, PWMB = 23, 24, 13
STBY = 22

# === Devices ===
ain1 = DigitalOutputDevice(AIN1, initial_value=False)
ain2 = DigitalOutputDevice(AIN2, initial_value=False)
bin1 = DigitalOutputDevice(BIN1, initial_value=False)
bin2 = DigitalOutputDevice(BIN2, initial_value=False)
stby = DigitalOutputDevice(STBY, initial_value=False)

# Software PWM; 1 kHz is a good starting point
pwmA = PWMOutputDevice(PWMA, frequency=1000, initial_value=0.0)
pwmB = PWMOutputDevice(PWMB, frequency=1000, initial_value=0.0)

def standby(enable: bool):
    stby.on() if enable else stby.off()

def coast():
    # Outputs off (high-Z) -> motor coasts
    pwmA.off(); pwmB.off()
    ain1.off(); ain2.off()
    bin1.off(); bin2.off()

def brake():
    # Short-brake: both inputs HIGH + PWM 100%
    ain1.on(); ain2.on()
    bin1.on(); bin2.on()
    pwmA.value = 1.0
    pwmB.value = 1.0

# >>> Swapped to match your physical "forward" <<<
def set_dir_forward():
    # A and B spin in the physical FORWARD direction for your chassis
    ain1.off(); ain2.on()
    bin1.off(); bin2.on()

def set_dir_reverse():
    ain1.on();  ain2.off()
    bin1.on();  bin2.off()

def drive(speed: float):
    # speed in [0.0, 1.0]
    s = max(0.0, min(1.0, speed))
    pwmA.value = s
    pwmB.value = s

def forward(seconds=2.0, speed=0.6):
    standby(True)
    set_dir_forward()
    drive(speed)
    sleep(seconds)
    brake(); sleep(0.3)
    coast()
    standby(False)

def reverse(seconds=2.0, speed=0.6):
    standby(True)
    set_dir_reverse()
    drive(speed)
    sleep(seconds)
    brake(); sleep(0.3)
    coast()
    standby(False)

if __name__ == "__main__":
    try:
        print("Forward…")
        forward(seconds=1.5, speed=0.6)
        sleep(0.5)
        print("Reverse…")
        reverse(seconds=1.5, speed=0.6)
        print("Done.")
    finally:
        coast()
        standby(False)