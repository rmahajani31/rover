import gpiod
import math, threading, time
from typing import Optional

# Utility clamp function
def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

# Base gpiod backend class
class _GpioBackendBase:
    def setup(self): ...
    def set_dir_left(self, forward: bool): ...
    def set_dir_right(self, forward: bool): ...
    def set_pwm_left(self, duty01: float): ...
    def set_pwm_right(self, duty01: float): ...
    def stby(self, high: bool): ...
    def shutdown(self): ...

# Class to define helper methods associated with gpiod lines
class _GpiodLine:
    def __init__(self, chip, offset, consumer="tb6612"):
        self._line = None
        self._chip = chip
        self._offset = offset
        self._consumer = consumer
        self._req = None
    
    def request_output(self, initial=0):
        """Configures the gpio line as an output line and reserves it for the consumer process"""
        line = self._chip.get_line(self._offset)
        line.request(consumer=self._consumer, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[initial])
        self._line = line
    
    def set_value(self, val: int):
        """Sets a gpio line to a value"""
        self._line.set_value(1 if val else 0)
    
    def close(self):
        """Closes a gpio line"""
        self._line.release()
        self._line = None

class _SoftPWM:
    """Software PWM on a gpiod line. Use moderate freq (e.g., 300–500 Hz)."""
    def __init__(self, line: _GpiodLine, freq_hz: float, start_duty: float = 0.0):
        self._line = line
        self._freq = max(1.0, float(freq_hz))
        self._duty = clamp01(start_duty)
        self._running = False
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    def set_duty(self, duty01: float):
        """Sets the new duty for the PWM cycle, effectively increasing or decreasing the speed of the motor"""
        with self._lock:
            self._duty = clamp01(duty01)

    def set_freq(self, freq_hz: float):
        """Sets the new frequency for the PWM cycle, effectively changing the speed at which PWM signals are transmitted"""
        with self._lock:
            self._freq = max(1.0, float(freq_hz))

    def start(self):
        """Starts the loop which controls the PWM cycles"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stops the PWM cycle loop from running"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)
            self._thread = None
        # ensure low
        self._line.set_value(0)

    def _loop(self):
        """simple two-level PWM; Python timing has jitter, keep freq moderate"""
        while self._running:
            with self._lock:
                duty = self._duty
                period = 1.0 / self._freq
            if duty <= 0.0:
                self._line.set_value(0)
                time.sleep(period)
                continue
            if duty >= 1.0:
                self._line.set_value(1)
                time.sleep(period)
                continue
            t_on = period * duty
            t_off = period - t_on
            self._line.set_value(1); time.sleep(t_on)
            self._line.set_value(0); time.sleep(t_off)

class GpiodBackend(_GpioBackendBase):
    """GPIO + software PWM using libgpiod."""
    def __init__(self, gpiochip: str, ain1, ain2, pwma, bin1, bin2, pwmb, stby, pwm_hz):
        self.chip = gpiod.Chip(gpiochip)
        self.ain1 = _GpiodLine(self.chip, ain1)
        self.ain2 = _GpiodLine(self.chip, ain2)
        self.bin1 = _GpiodLine(self.chip, bin1)
        self.bin2 = _GpiodLine(self.chip, bin2)
        self.stby = _GpiodLine(self.chip, stby)
        self.pwma = _GpiodLine(self.chip, pwma)
        self.pwmb = _GpiodLine(self.chip, pwmb)
        self._pwmA = _SoftPWM(self.pwma, pwm_hz, 0.0)
        self._pwmB = _SoftPWM(self.pwmb, pwm_hz, 0.0)

    def setup(self):
        """Set all the lines connected to the motor driver as output lines"""
        # request outputs
        for ln in [self.ain1, self.ain2, self.bin1, self.bin2, self.stby, self.pwma, self.pwmb]:
            ln.request_output(initial=0)
        # start PWM threads
        self._pwmA.start()
        self._pwmB.start()

    def set_dir_left(self, forward: bool):
        """Set the direction of the left motor"""
        self.ain1.set_value(0 if forward else 1)
        self.ain2.set_value(1 if forward else 0)

    def set_dir_right(self, forward: bool):
        """Set the direction of the right motor"""
        self.bin1.set_value(0 if forward else 1)
        self.bin2.set_value(1 if forward else 0)
        # self.bin1.set_value(1 if forward else 0)
        # self.bin2.set_value(0 if forward else 1)

    def set_pwm_left(self, duty01: float):
        """Set the duty cycle of the left motor"""
        self._pwmA.set_duty(clamp01(duty01))

    def set_pwm_right(self, duty01: float):
        """Set the duty cycle of the right motor"""
        self._pwmB.set_duty(clamp01(duty01))

    def set_stby(self, high: bool):
        """Set the stby line value"""
        self.stby.set_value(1 if high else 0)

    def shutdown(self):
        """Stop the motors"""
        self._pwmA.stop()
        self._pwmB.stop()
        for ln in [self.ain1, self.ain2, self.bin1, self.bin2, self.stby]:
            ln.set_value(0)

if __name__ == "__main__": 
    ain1 = 5
    ain2 = 6
    pwma = 12
    bin1 = 16
    bin2 = 24
    pwmb = 13
    stby = 25

    motor_driver = GpiodBackend(gpiochip="gpiochip4", ain1=ain1, ain2=ain2, pwma=pwma, bin1=bin1, bin2=bin2, pwmb=pwmb, stby=stby, pwm_hz=300)
    motor_driver.setup()
    motor_driver.set_stby(True)
    motor_driver.set_dir_left(True)
    motor_driver.set_dir_right(True)


    motor_driver.set_pwm_left(0.5)
    time.sleep(5)
    motor_driver.set_pwm_left(0.0)
    motor_driver.set_stby(False)
    
    time.sleep(1)

    motor_driver.set_stby(True)
    motor_driver.set_pwm_right(0.5)
    time.sleep(5)
    motor_driver.set_pwm_right(0.0)
    motor_driver.set_stby(False)