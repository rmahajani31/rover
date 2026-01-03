#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

from .encoder_backend import EncoderBackend


class EncoderTicksNode(Node):
    def __init__(self):
        super().__init__("encoder_ticks")

        # Parameters
        self.declare_parameter("gpiochip", "gpiochip4")
        self.declare_parameter("publish_rate_hz", 50.0)

        gpiochip = self.get_parameter("gpiochip").value
        self.publish_rate = float(self.get_parameter("publish_rate_hz").value)

        # Your wiring:
        self.backend = EncoderBackend(
            gpiochip=gpiochip,
            left_a=17, left_b=27,
            right_a=23, right_b=22
        )
        self.backend.setup()

        self.left_pub = self.create_publisher(Int64, "/left_ticks", 10)
        self.right_pub = self.create_publisher(Int64, "/right_ticks", 10)

        self._stop = False
        self._lock = threading.Lock()

        # Poll thread: watches GPIO edges and updates counts
        threading.Thread(target=self.backend.poll_forever, daemon=True).start()

        # Publish timer: publishes counts at a fixed rate (no per-edge printing)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)

        self.get_logger().info("EncoderTicksNode started (publishing /left_ticks and /right_ticks)")

    def _poll_loop(self):
        while rclpy.ok() and not self._stop:
            with self._lock:
                self.backend.poll_once(timeout_s=0.005)
            time.sleep(0.001)

    def _publish(self):
        with self._lock:
            lt = self.backend.left_ticks
            rt = self.backend.right_ticks

        self.left_pub.publish(Int64(data=lt))
        self.right_pub.publish(Int64(data=rt))

    def destroy_node(self):
        self._stop = True
        try:
            self._thread.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = EncoderTicksNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()