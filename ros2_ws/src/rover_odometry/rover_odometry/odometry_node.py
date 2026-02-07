import time
import math
from .pinpoint_i2c import PinpointI2C

import rclpy
from rclpy.node import Node

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

class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry")

        # Declare Parameters
        self.declare_parameter("bus", 1)
        self.declare_parameter("addr", 0x31)
        self.declare_parameter("endian", "little")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("pod_offsets_mm", [160.87, -201.3])
        self.declare_parameter("encoder_directions", [True, True])
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # Initialize I2C parameters
        bus = self.get_parameter("bus").value
        addr = self.get_parameter("addr").value
        endian = self.get_parameter("endian").value
        self.i2c = PinpointI2C(bus, addr, endian)

        # Initialize other parameters
        self.pod_offsets_mm = self.get_parameter("pod_offsets_mm").value
        self.encoder_directions = self.get_parameter("encoder_directions").value
        self.rate = float(self.get_parameter("publish_rate_hz").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        # Initialize register names and formats
        reg_to_name = {
            1: " Device ID",
            2: " Version",
            3: " Status",
            4: " Control",
            6: " X Encoder",
            7: " Y Encoder",
            8: " X Position",
            9: " Y Position",
            10: " Yaw",
            11: " X Velocity",
            12: " Y Velocity",
            13: " Yaw Velocity",
            14: " Ticks per mm",
            15: " X Offset",
            16: " Y Offset",
            17: " Yaw Offset",
        }
        self.reg_to_format = {
            1: "I",
            2: "I",
            3: "I",
            4: "I",
            6: "I",
            7: "I",
            8: "f",
            9: "f",
            10: "f",
            11: "f",
            12: "f",
            13: "f",
            14: "f",
            15: "f",
            16: "f",
            17: "f",
        }
        self.name_to_reg = {
            name.strip(): reg for reg, name in reg_to_name.items()
        }

        # Initialize I2C device
        self.i2c.reset_pos_and_imu()
        time.sleep(1)
        self.i2c.set_pod_offsets_mm(self.pod_offsets_mm[0], self.pod_offsets_mm[1])
        time.sleep(1)
        self.i2c.set_encoder_directions(self.encoder_directions[0], self.encoder_directions[1])
        time.sleep(1)

        # Initialize state
        self.x = self.i2c.read_f32(self.name_to_reg["X Position"])
        self.y = self.i2c.read_f32(self.name_to_reg["Y Position"])
        self.yaw = self.i2c.read_f32(self.name_to_reg["Yaw"])
        self.vx = self.i2c.read_f32(self.name_to_reg["X Velocity"])
        self.vy = self.i2c.read_f32(self.name_to_reg["Y Velocity"])
        self.vh = self.i2c.read_f32(self.name_to_reg["Yaw Velocity"])

        # Initialize publishers and timer
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_br = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.rate, self._update)
        self.get_logger().info(
            f"odometry started. x={self.x}m, y={self.y}m, yaw={self.yaw}rad"
        )
    
    def _update(self):
        self.x = self.i2c.read_f32(self.name_to_reg["X Position"])
        self.y = self.i2c.read_f32(self.name_to_reg["Y Position"])
        self.yaw = self.i2c.read_f32(self.name_to_reg["Yaw"])
        self.vx = self.i2c.read_f32(self.name_to_reg["X Velocity"])
        self.vy = self.i2c.read_f32(self.name_to_reg["Y Velocity"])
        self.vh = self.i2c.read_f32(self.name_to_reg["Yaw Velocity"])

        # ---- Publish TF: odom -> base_link ----
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x / 1000.0
        tf_msg.transform.translation.y = self.y / 1000.0
        tf_msg.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(wrap_pi(self.yaw))
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_br.sendTransform(tf_msg)

        # ---- Publish /odom message ----
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x / 1000.0
        odom.pose.pose.position.y = self.y / 1000.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx / 1000.0
        odom.twist.twist.linear.y = self.vy / 1000.0
        odom.twist.twist.angular.z = self.vh

        # Simple covariance placeholders (Phase 0 level)
        # Increase yaw uncertainty when turning
        xy_var = 0.01 + 0.05 * abs(self.vh)
        yaw_var = 0.02 + 0.10 * abs(self.vh)

        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = xy_var       # x
        odom.pose.covariance[7] = xy_var       # y
        odom.pose.covariance[35] = yaw_var     # yaw
        self.odom_pub.publish(odom)
    
def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()






