import time
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


def clip11(x: float) -> float:
    """Clamp a value to [-1.0, 1.0]."""
    return -1.0 if x < -1.0 else (1.0 if x > 1.0 else x)


def clamp01(x: float) -> float:
    """Clamp a value to [0.0, 1.0]."""
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def apply_expo(x: float, expo: float) -> float:
    """Blend linear and cubic response for finer control near stick center."""
    expo = clamp01(expo)
    return ((1.0 - expo) * x) + (expo * x * x * x)


def step_toward(current: float, target: float, max_step: float) -> float:
    """Move current toward target without exceeding max_step."""
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


class AdapterNode(Node):
    def __init__(self):
        super().__init__('adapter_node')
        
        # ===== Parameters =====
        # Axes
        self.declare_parameter('throttle_axis', 1)         # e.g., left stick Y
        self.declare_parameter('steer_axis', 3)            # e.g., right stick X
        self.declare_parameter('deadzone', 0.06)
        self.declare_parameter('max_linear_x', 0.35)
        self.declare_parameter('max_angular_z', 0.8)
        self.declare_parameter('throttle_expo', 0.5)
        self.declare_parameter('steer_expo', 0.6)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('accel_rate_linear', 0.6)
        self.declare_parameter('decel_rate_linear', 1.5)
        self.declare_parameter('accel_rate_angular', 1.2)
        self.declare_parameter('decel_rate_angular', 2.5)

        # Initial polarity (can be toggled at runtime via buttons)
        self.declare_parameter('invert_throttle', False)
        self.declare_parameter('invert_steer', False)
        self.declare_parameter('start_swapped', False)

        # Buttons (indices)
        self.declare_parameter('btn_estop', [0])           # list → any press triggers E-stop
        self.declare_parameter('btn_swap', -1)
        self.declare_parameter('btn_inv_throttle', -1)
        self.declare_parameter('btn_inv_steer', -1)

        # Read parameters
        self.throttle_axis = int(self.get_parameter('throttle_axis').value)
        self.steer_axis = int(self.get_parameter('steer_axis').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.max_linear_x = float(self.get_parameter('max_linear_x').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.throttle_expo = float(self.get_parameter('throttle_expo').value)
        self.steer_expo = float(self.get_parameter('steer_expo').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.accel_rate_linear = float(self.get_parameter('accel_rate_linear').value)
        self.decel_rate_linear = float(self.get_parameter('decel_rate_linear').value)
        self.accel_rate_angular = float(self.get_parameter('accel_rate_angular').value)
        self.decel_rate_angular = float(self.get_parameter('decel_rate_angular').value)

        self.invert_throttle = bool(self.get_parameter('invert_throttle').value)
        self.invert_steer = bool(self.get_parameter('invert_steer').value)
        self.swapped = bool(self.get_parameter('start_swapped').value)

        self.btn_estop: List[int] = list(self.get_parameter('btn_estop').value)
        self.btn_swap = int(self.get_parameter('btn_swap').value)
        self.btn_inv_throttle = int(self.get_parameter('btn_inv_throttle').value)
        self.btn_inv_steer = int(self.get_parameter('btn_inv_steer').value)

        # Define publishers and subscribers

        # cmd_vel will be the topic which converts joystick movements into motor velocity commands
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # estop will be the topic which initiates an emergency stop
        # Note the transient local setting indicates that the final message is given to subscribers even if it was published before they subscribed
        # This is the latched behavior which is important for e-stop to indicate that the robot needs to be stopped
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_estop = self.create_publisher(Bool, '/estop', latched_qos)
        self.estop_state = False

        # Subscribe to /joy to track joystick actions
        self.sub_joy = self.create_subscription(Joy, '/joy', self.on_joy, 10)

        # Smoothed command state
        self.desired_linear_x = 0.0
        self.desired_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.last_publish_time = time.monotonic()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        # Internal state for edge detection
        self.prev_buttons: Optional[List[int]] = None

        # Log initial config
        self._log_mapping()
    
    def _log_mapping(self):
        """Log all the initial parameters"""
        self.get_logger().info(
            f'axes: throttle={self.throttle_axis} steer={self.steer_axis} | '
            f'invert(thr)={self.invert_throttle} invert(steer)={self.invert_steer} | '
            f'swapped={self.swapped} deadzone={self.deadzone:.3f}'
        )
        self.get_logger().info(
            f'buttons: estop={self.btn_estop} swap={self.btn_swap} '
            f'inv_thr={self.btn_inv_throttle} inv_steer={self.btn_inv_steer}'
        )
        self.get_logger().info(
            f'limits: max_linear_x={self.max_linear_x:.3f} max_angular_z={self.max_angular_z:.3f} '
            f'expo(thr)={self.throttle_expo:.2f} expo(steer)={self.steer_expo:.2f}'
        )
        self.get_logger().info(
            f'smoothing: publish_rate_hz={self.publish_rate_hz:.1f} '
            f'accel_linear={self.accel_rate_linear:.2f} decel_linear={self.decel_rate_linear:.2f} '
            f'accel_angular={self.accel_rate_angular:.2f} decel_angular={self.decel_rate_angular:.2f}'
        )
    
    def _edge_pressed(self, idx: int, buttons: List[int]) -> bool:
        """True on rising edge (0→1). idx<0 means 'disabled'."""
        if idx < 0:
            return False
        if self.prev_buttons is None:
            return False
        prev = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
        now  = buttons[idx] if idx < len(buttons) else 0
        return prev == 0 and now == 1

    def _any_edge_pressed(self, indices: List[int], buttons: List[int]) -> bool:
        "Check if any of the buttons associated with the indices have rising edges"
        return any(self._edge_pressed(i, buttons) for i in indices if i >= 0)

    def _apply_deadzone(self, x: float) -> float:
        """Enforce a centered deadzone."""
        return 0.0 if abs(x) < self.deadzone else x

    def _scale_axis(self, x: float, expo: float, max_value: float) -> float:
        """Shape and scale a joystick axis to the command range."""
        shaped = apply_expo(self._apply_deadzone(clip11(x)), expo)
        return shaped * max_value

    def _step_command(self, current: float, target: float, accel_rate: float, decel_rate: float, dt: float) -> float:
        """Rate-limit command changes and prefer quicker decay toward zero."""
        same_direction = (current == 0.0) or (target == 0.0) or ((current > 0.0) == (target > 0.0))
        reducing_magnitude = abs(target) < abs(current)
        use_decel = reducing_magnitude or not same_direction
        max_step = max(0.0, (decel_rate if use_decel else accel_rate) * dt)
        return step_toward(current, target, max_step)

    def _publish_twist(self, linear_x: float, angular_z: float):
        """Publish the current command to the firmware-facing /cmd_vel topic."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = linear_x
        twist.twist.angular.z = angular_z
        self.pub_cmd.publish(twist)

    def on_joy(self, msg: Joy):
        """Callback on any joystick actions"""
        axes = msg.axes
        buttons = msg.buttons

        # Block to manage state when the e-stop button is pressed
        if self._any_edge_pressed(self.btn_estop, buttons):
            self.get_logger().warn('E-STOP pressed! Publishing latched self.estop_state and zeroing command.')
            self.estop_state = not self.estop_state
            self.pub_estop.publish(Bool(data=self.estop_state))
            self.desired_linear_x = 0.0
            self.desired_angular_z = 0.0
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0
            self._publish_twist(0.0, 0.0)
            self.prev_buttons = buttons[:]  # update edge state
            return  # do not process command on this frame
        
        # Block of code to handle throttle steer axis swap
        if self._edge_pressed(self.btn_swap, buttons):
            self.swapped = not self.swapped
            self.get_logger().info(f'SWAP toggled → swapped={self.swapped}')
        
        # Block of code to handle throttle invert
        if self._edge_pressed(self.btn_inv_throttle, buttons):
            self.invert_throttle = not self.invert_throttle
            self.get_logger().info(f'Invert throttle → {self.invert_throttle}')
        
        # Block of code to handle steer invert
        if self._edge_pressed(self.btn_inv_steer, buttons):
            self.invert_steer = not self.invert_steer
            self.get_logger().info(f'Invert steer → {self.invert_steer}')
        
        # Map axes → (throttle, steer)
        def safe_get(idx: int) -> float:
            return axes[idx] if 0 <= idx < len(axes) else 0.0

        # Get the axis values and swap them if the swap button has been pressed
        ax_t = self.throttle_axis
        ax_s = self.steer_axis
        if self.swapped:
            ax_t, ax_s = ax_s, ax_t
        
        throttle = safe_get(ax_t)
        steer = safe_get(ax_s)

        # Invert the throttle and steer values if the respective buttons have been pressed
        if self.invert_throttle:
            throttle = -throttle
        if self.invert_steer:
            steer = -steer

        if self.estop_state:
            self.desired_linear_x = 0.0
            self.desired_angular_z = 0.0
        else:
            self.desired_linear_x = self._scale_axis(throttle, self.throttle_expo, self.max_linear_x)
            self.desired_angular_z = self._scale_axis(steer, self.steer_expo, self.max_angular_z)

        # Save for edge detection next time
        self.prev_buttons = buttons[:]

    def on_timer(self):
        """Publish smoothed commands to the firmware at a fixed rate."""
        now = time.monotonic()
        dt = max(1e-3, now - self.last_publish_time)
        self.last_publish_time = now

        self.current_linear_x = self._step_command(
            self.current_linear_x,
            self.desired_linear_x,
            self.accel_rate_linear,
            self.decel_rate_linear,
            dt,
        )
        self.current_angular_z = self._step_command(
            self.current_angular_z,
            self.desired_angular_z,
            self.accel_rate_angular,
            self.decel_rate_angular,
            dt,
        )
        self._publish_twist(self.current_linear_x, self.current_angular_z)

def main(args=None):
    try:
        with rclpy.init(args=args):
            adapter_node = AdapterNode()

            rclpy.spin(adapter_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        adapter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
