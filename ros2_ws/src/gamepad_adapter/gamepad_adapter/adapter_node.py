import math
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class AdapterNode(Node):
    def __init__(self):
        super().__init__('adapter_node')
        
        # ===== Parameters =====
        # Axes
        self.declare_parameter('throttle_axis', 1)         # e.g., left stick Y
        self.declare_parameter('steer_axis', 3)            # e.g., right stick X
        self.declare_parameter('deadzone', 0.06)

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

        self.invert_throttle = bool(self.get_parameter('invert_throttle').value)
        self.invert_steer = bool(self.get_parameter('invert_steer').value)
        self.swapped = bool(self.get_parameter('start_swapped').value)

        self.btn_estop: List[int] = list(self.get_parameter('btn_estop').value)
        self.btn_swap = int(self.get_parameter('btn_swap').value)
        self.btn_inv_throttle = int(self.get_parameter('btn_inv_throttle').value)
        self.btn_inv_steer = int(self.get_parameter('btn_inv_steer').value)

        # Define publishers and subscribers

        # cmd_vel will be the topic which converts joystick movements into motor velocity commands
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

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

    def on_joy(self, msg: Joy):
        """Callback on any joystick actions"""
        axes = msg.axes
        buttons = msg.buttons

        # Block to manage state when the e-stop button is pressed
        if self._any_edge_pressed(self.btn_estop, buttons):
            self.get_logger().warn('E-STOP pressed! Publishing latched self.estop_state and zeroing command.')
            self.estop_state = not self.estop_state
            self.pub_estop.publish(Bool(data=self.estop_state))
            # Also publish a zeroed Twist immediately
            self.pub_cmd.publish(Twist())
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
        
        # clip the axis values to be in the range [-1.0, 1.0]
        def clip(x: float) -> float:
            return -1.0 if x < -1.0 else (1.0 if x > 1.0 else x)

        # Enforce a deadzone on throttle and steer where the robot will not move
        def _apply_deadzone(x: float) -> float:
            return 0.0 if abs(x) < self.deadzone else x

        # Get the axis values and swap them if the swap button has been pressed
        ax_t = self.throttle_axis
        ax_s = self.steer_axis
        if self.swapped:
            ax_t, ax_s = ax_s, ax_t
        
        # After clipping and applying the deadzone get the throttle and steer values
        throttle = _apply_deadzone(clip(safe_get(ax_t)))
        steer = _apply_deadzone(clip(safe_get(ax_s)))

        # Invert the throttle and steer values if the respective buttons have been pressed
        if self.invert_throttle:
            throttle = -throttle
        if self.invert_steer:
            steer = -steer
        
        # Publish a twist message to cmd_vel to drive the motor, linear.x is throttle and angular.z is steer
        twist = Twist()
        twist.linear.x  = throttle
        twist.angular.z = steer
        self.pub_cmd.publish(twist)

        # Save for edge detection next time
        self.prev_buttons = buttons[:]

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
