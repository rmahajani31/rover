import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    """Convert 2D yaw to quaternion (x,y,z,w)."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def wrap_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__("encoder_odom")

        # ---- Parameters you will tune ----
        self.declare_parameter("wheel_radius_m", 0.033)        # radius in meters
        self.declare_parameter("wheel_separation_m", 0.16)     # track width in meters
        self.declare_parameter("ticks_per_rev", 3840.0)        # x4 for your DFrobot TT encoder
        self.declare_parameter("publish_rate_hz", 50.0)

        self.declare_parameter("left_ticks_topic", "/left_ticks")
        self.declare_parameter("right_ticks_topic", "/right_ticks")

        # Frame names (standard)
        self.declare_parameter("odom_frame", "wheel_odom")
        self.declare_parameter("base_frame", "base_link")

        # If your tick sign is backwards for a wheel, set -1.0
        self.declare_parameter("left_dir", 1.0)
        self.declare_parameter("right_dir", 1.0)
        
        # Deadband to prevent creep when stationary (in ticks)
        self.declare_parameter("tick_deadband", 2.0)

        self.R = float(self.get_parameter("wheel_radius_m").value)
        self.B = float(self.get_parameter("wheel_separation_m").value)
        self.N = float(self.get_parameter("ticks_per_rev").value)
        self.rate = float(self.get_parameter("publish_rate_hz").value)

        self.left_topic = str(self.get_parameter("left_ticks_topic").value)
        self.right_topic = str(self.get_parameter("right_ticks_topic").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.left_dir = float(self.get_parameter("left_dir").value)
        self.right_dir = float(self.get_parameter("right_dir").value)
        self.tick_deadband = float(self.get_parameter("tick_deadband").value)

        # ---- State ----
        self.left_ticks = None
        self.right_ticks = None
        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_time = self.get_clock().now()

        # ---- ROS I/O ----
        self.create_subscription(Int64, self.left_topic, self._on_left, 10)
        self.create_subscription(Int64, self.right_topic, self._on_right, 10)

        self.odom_pub = self.create_publisher(Odometry, "/wheel_odom", 10)
        self.tf_br = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.rate, self._update)

        self.get_logger().info(
            f"encoder_odom started. R={self.R}m, B={self.B}m, N={self.N} ticks/rev"
        )

    def _on_left(self, msg: Int64):
        self.left_ticks = msg.data

    def _on_right(self, msg: Int64):
        self.right_ticks = msg.data

    def _update(self):
        if self.left_ticks is None or self.right_ticks is None:
            return

        # Initialize on first tick reception
        if self.prev_left_ticks is None:
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            self.prev_time = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # Tick deltas (apply direction sign)
        dL_ticks = (self.left_ticks - self.prev_left_ticks) * self.left_dir
        dR_ticks = (self.right_ticks - self.prev_right_ticks) * self.right_dir

        # Apply deadband to prevent creep from encoder noise when stationary
        # If both wheels have very small changes, treat as zero movement
        if abs(dL_ticks) < self.tick_deadband and abs(dR_ticks) < self.tick_deadband:
            dL_ticks = 0.0
            dR_ticks = 0.0

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.prev_time = now

        # Convert to meters
        meters_per_tick = (2.0 * math.pi * self.R) / self.N
        dL = dL_ticks * meters_per_tick
        dR = dR_ticks * meters_per_tick

        # Differential drive integration
        ds = 0.5 * (dR + dL)
        dyaw = (dR - dL) / self.B

        # Midpoint integration
        self.x += ds * math.cos(self.yaw + 0.5 * dyaw)
        self.y += ds * math.sin(self.yaw + 0.5 * dyaw)
        self.yaw = wrap_pi(self.yaw + dyaw)

        # Velocities (for /odom twist)
        v = ds / dt
        w = dyaw / dt

        # ---- Publish TF: odom -> base_link ----
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        # self.tf_br.sendTransform(tf_msg)

        # ---- Publish /odom message ----
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Simple covariance placeholders (Phase 0 level)
        # Increase yaw uncertainty when turning
        xy_var = 0.01 + 0.05 * abs(w)
        yaw_var = 0.02 + 0.10 * abs(w)

        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = xy_var       # x
        odom.pose.covariance[7] = xy_var       # y
        odom.pose.covariance[35] = yaw_var     # yaw

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()