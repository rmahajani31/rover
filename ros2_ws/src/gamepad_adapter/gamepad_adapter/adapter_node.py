import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class AdapterNode(Node):
    def __init__(self):
        super().__init__('adapter_node')
        self.get_logger().info('Hello ROS2')
        

def main(args=None):
    try:
        with rclpy.init(args=args):
            adapter_node = AdapterNode()

            rclpy.spin(adapter_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
