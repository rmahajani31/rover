import math, threading, time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from drive_msgs.msg import WheelCommand

import gpiod

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
        # self.bin1.set_value(0 if forward else 1)
        # self.bin2.set_value(1 if forward else 0)
        self.bin1.set_value(1 if forward else 0)
        self.bin2.set_value(0 if forward else 1)

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

# Main node for the motor driver
class TB6612Driver(Node):
    """
    Subscribes:
      /wheel_cmd (drive_msgs/WheelCommand)
      /estop     (std_msgs/Bool)  [latched QoS]
    Params:
      gpiochip (string, e.g., "gpiochip4" on Pi 5 RP1)  <-- NEW
      ain1, ain2, pwma, bin1, bin2, pwmb, stby (line offsets)
      pwm_frequency_hz
    """

    def __init__(self):
        super().__init__('tb6612_driver_node')

        # ---- Parameters
        self.declare_parameter('gpiochip', 'gpiochip0')   # Pi 5 often 'gpiochip4' for RP1
        self.declare_parameter('ain1', 5)
        self.declare_parameter('ain2', 6)
        self.declare_parameter('pwma', 12)
        self.declare_parameter('bin1', 23)
        self.declare_parameter('bin2', 24)
        self.declare_parameter('pwmb', 13)
        self.declare_parameter('stby', 22)               # set -1 if not wired
        self.declare_parameter('pwm_frequency_hz', 400)  # start moderate for SW PWM

        P = self.get_parameter
        chip_name = str(P('gpiochip').value)
        ain1 = int(P('ain1').value)
        ain2 = int(P('ain2').value)
        pwma = int(P('pwma').value)
        bin1 = int(P('bin1').value)
        bin2 = int(P('bin2').value)
        pwmb = int(P('pwmb').value)
        stby = int(P('stby').value)
        pwm_hz = int(P('pwm_frequency_hz').value)

        # ---- Backend setup
        self.backend = GpiodBackend(chip_name, ain1, ain2, pwma, bin1, bin2, pwmb, stby, pwm_hz)
        self.get_logger().info(f'Using gpiod backend on {chip_name} @ {pwm_hz} Hz')
        self.backend.setup()
        self.safe_idle()

        # ---- Topic Subscription Setup
        self.estop_active = False

        estop_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.sub_estop = self.create_subscription(Bool, '/estop', self.on_estop, estop_qos)
        self.sub_cmd   = self.create_subscription(WheelCommand, '/wheel_cmd', self.on_wheel_cmd, 10)

        self.get_logger().info('tb6612_driver ready')

    def safe_idle(self):
        """Stops the rover"""
        self.backend.set_pwm_left(0.0)
        self.backend.set_pwm_right(0.0)
        self.backend.set_stby(False)

    # ---- Callbacks
    def on_estop(self, msg: Bool):
        """Checks if estop has been pressed and stops the rover if it has been pressed"""
        self.estop_active = bool(msg.data)
        if self.estop_active:
            self.safe_idle()
            self.get_logger().warn('E-STOP active → outputs zero, STBY low')
        else:
            self.get_logger().info('E-STOP cleared')

    def on_wheel_cmd(self, msg: WheelCommand):
        """Sets the directions and duty cycles based on the values published by the arcade mixer"""
        if self.estop_active:
            return
            
        self.backend.set_stby(True)
        self.backend.set_dir_left(not bool(msg.left_forward))
        self.backend.set_dir_right(not bool(msg.right_forward))
        self.backend.set_pwm_left(clamp01(float(msg.left_pwm)))
        self.backend.set_pwm_right(clamp01(float(msg.right_pwm)))

        # When there is no duty input stop the rover, so that there isn't any creep
        if msg.left_pwm <= 0.0 and msg.right_pwm <= 0.0:
            self.safe_idle()

    # ---- Cleanup
    def destroy_node(self):
        """Stops the Gpiod Backend and destroys the node"""
        try:
            self.backend.shutdown()
        finally:
            super().destroy_node()

def main(args=None):
    try:
        with rclpy.init(args=args):
            tb6612_driver_node = TB6612Driver()

            rclpy.spin(tb6612_driver_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        tb6612_driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()