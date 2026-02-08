import math, time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from drive_msgs.msg import WheelCommand

def clip11(x: float) -> float:
    return -1.0 if x < -1.0 else (1.0 if x > 1.0 else x)

def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

def with_floor(u: float, floor: float) -> float:
    if abs(u) < 1e-6:
        return 0.0
    return math.copysign(floor + (1.0 - floor) * abs(u), u)

class ArcadeMixerNode(Node):
    def __init__(self):
        super().__init__('arcade_mixer_node')

        # Defining params
        self.declare_parameter('invert_right', True)         # INVERT_B
        self.declare_parameter('trim_right', 0.0)            # TRIM_B
        self.declare_parameter('max_speed', 0.85)            # MAX_SPEED
        self.declare_parameter('smooth', 0.22)               # SMOOTH (0..1)
        self.declare_parameter('idle_timeout_s', 0.8)        # IDLE_TIMEOUT
        self.declare_parameter('deadzone', 0.06)             # DEADZONE
        self.declare_parameter('min_pwm_drive', 0.18)        # MIN_PWM_DRIVE
        self.declare_parameter('min_pwm_pivot', 0.22)        # MIN_PWM_PIVOT
        self.declare_parameter('zero_hold', 0.10)            # ZERO_HOLD
        self.declare_parameter('publish_rate_hz', 50.0)      # fixed update

        self.invert_right   = bool(self.get_parameter('invert_right').value)
        self.trim_right     = float(self.get_parameter('trim_right').value)
        self.max_speed      = float(self.get_parameter('max_speed').value)
        self.smooth         = float(self.get_parameter('smooth').value)
        self.idle_timeout_s = float(self.get_parameter('idle_timeout_s').value)
        self.deadzone       = float(self.get_parameter('deadzone').value)
        self.min_pwm_drive  = float(self.get_parameter('min_pwm_drive').value)
        self.min_pwm_pivot  = float(self.get_parameter('min_pwm_pivot').value)
        self.zero_hold      = float(self.get_parameter('zero_hold').value)
        hz                  = float(self.get_parameter('publish_rate_hz').value)

        # State
        self.estop_active: bool = False
        self.last_cmd_time: float = 0.0

        self.last_throttle: float = 0.0
        self.last_steer: float = 0.0

        # Targets (signed -1..1 after floors/normalization)
        self.tgtL: float = 0.0
        self.tgtR: float = 0.0

        # Smoothed magnitudes 0..1 (post max_speed)
        self.curL: float = 0.0
        self.curR: float = 0.0

        # Last directions (+1/-1) actually applied
        self.dirL: int = +1
        self.dirR: int = +1

        # I/O
        self.sub_cmd  = self.create_subscription(Twist, '/cmd_arcade', self.on_cmd, 10)
        self.sub_stop = self.create_subscription(Bool, '/estop', self.on_estop, 10)
        self.pub_wcmd = self.create_publisher(WheelCommand, '/wheel_cmd', QoSProfile(depth=10))

        # Publish to wheels at rate determined by hz
        self.timer = self.create_timer(1.0 / hz, self.on_timer)
        self.get_logger().info('arcade_mixer ready')

    def on_estop(self, msg: Bool):
        """Set everything to 0 if the estop button has been pressed"""
        self.estop_active = bool(msg.data)
        if self.estop_active:
            # Immediately zero targets and state
            self.tgtL = self.tgtR = 0.0
            self.curL = self.curR = 0.0
            self.get_logger().warn('E-STOP active → outputs forced to zero')
    
    def on_cmd(self, msg: Twist):
        """Set the wheel commands based on throttle and steer messages prior to smoothing"""
        # Record receipt time for idle timeout
        self.last_cmd_time = time.time()

        if self.estop_active:
            # Ignore commands while in estop
            self.tgtL = self.tgtR = 0.0
            return

        # Get the throttle and steer values from the twist message
        throttle = float(clip11(msg.linear.x))
        steer    = float(clip11(msg.angular.z))

        self.last_throttle = throttle
        self.last_steer = steer

        # If fully centered, zero targets
        if abs(throttle) < self.deadzone and abs(steer) < self.deadzone:
            self.tgtL = self.tgtR = 0.0
            return

        # Mix the throttle and steer values (arcade mixing) to get the right wheel commands before smoothing
        dir_sign = 1.0 if throttle >= 0.0 else -1.0
        f = throttle
        steer_eff = steer * dir_sign

        l = f + steer_eff
        r = f - steer_eff

        # Circular normalization to keep the proportions between l and r the same while clipping in the range [-1, 1]
        scale = max(1.0, abs(f) + abs(steer))
        l = clip11(l / scale)
        r = clip11(r / scale)

        # Floors which ensure that the l and r values take on certain minimum values
        if abs(f) >= self.deadzone:
            if l != 0.0:
                l = with_floor(l, self.min_pwm_drive)
            if r != 0.0:
                r = with_floor(r, self.min_pwm_drive)
        else:
            if l != 0.0:
                l = with_floor(l, self.min_pwm_pivot)
            if r != 0.0:
                r = with_floor(r, self.min_pwm_pivot)
        
        # Targets are signed [-1,1] after floors
        self.tgtL = l
        self.tgtR = r
    
    def on_timer(self):
        """Publish the final wheel message after processing the latest velocity commands for the left and right motors"""
        now = time.time()

        # Idle timeout or estop → hard zero
        if self.estop_active or (now - self.last_cmd_time > self.idle_timeout_s):
            self.tgtL = self.tgtR = 0.0
        
        # Prevent the rover from suddenly changing directions and instead wait until the PWM is less than zero_hold
        desired_dir_L = +1 if self.tgtL >= 0.0 else -1
        desired_dir_R = +1 if self.tgtR >= 0.0 else -1
        magL = abs(self.tgtL)
        magR = abs(self.tgtR)

        out_dir_L = desired_dir_L
        out_dir_R = desired_dir_R
        goalL = magL
        goalR = magR

        if desired_dir_L != self.dirL and self.curL > self.zero_hold:
            out_dir_L = self.dirL
            goalL = 0.0
        if desired_dir_R != self.dirR and self.curR > self.zero_hold:
            out_dir_R = self.dirR
            goalR = 0.0

        # Smoothing to prevent jerky movement
        # self.curL = (1.0 - self.smooth) * self.curL + self.smooth * (goalL * self.max_speed)
        # self.curR = (1.0 - self.smooth) * self.curR + self.smooth * (goalR * self.max_speed)

        self.curL = goalL * self.max_speed
        self.curR = goalR * self.max_speed

        # Update last applied directions
        self.dirL = out_dir_L
        self.dirR = out_dir_R

        # Set left and right forward values for the wheel msg
        left_forward  = (self.dirL >= 0)
        right_forward = (self.dirR >= 0)

        # Apply invert_right (direction flip only)
        if self.invert_right:
            right_forward = not right_forward
        
        # Option to add a bias to the right motor in case it is weaker and we want to prevent drift
        left_pwm  = clamp01(self.curL)
        right_pwm = clamp01(self.curR + self.trim_right)

        # Create and publish the wheel msg
        msg = WheelCommand()
        msg.left_pwm = float(left_pwm)
        msg.right_pwm = float(right_pwm)
        msg.left_forward = bool(left_forward)
        msg.right_forward = bool(right_forward)

         # Debug (log at 5 Hz to avoid spam on a 50 Hz timer)
        # if not hasattr(self, "_dbg_i"):
        #     self._dbg_i = 0
        # self._dbg_i += 1
        # if self._dbg_i % 10 == 0:
        #     self.get_logger().info(
        #         f"in=(thr={self.last_throttle:+.3f}, steer={self.last_steer:+.3f}) "
        #         f"tgt=(L={self.tgtL:+.3f}, R={self.tgtR:+.3f}) "
        #         f"goal=(L={goalL:.3f}, R={goalR:.3f}) "
        #         f"cur=(L={self.curL:.3f}, R={self.curR:.3f}) "
        #         f"pwm=(L={left_pwm:.3f}, R={right_pwm:.3f}) "
        #         f"dir=(L={self.dirL:+d}, R={self.dirR:+d})"
        #     )

        self.pub_wcmd.publish(msg)
    
def main(args=None):
    try:
        with rclpy.init(args=args):
            arcade_mixer_node = ArcadeMixerNode()

            rclpy.spin(arcade_mixer_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        arcade_mixer_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()