#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ShapeNode(Node):
    def __init__(self):
        super().__init__("shape_node")
        self.publisher = self.create_publisher(String, "shape_topic", 10)
        self.get_logger().info("ShapeNode started.\n")
        self.timer = self.create_timer(0.5, self.ask_user)

    def ask_user(self):
        shape = input("Which shape do you want?\n1.Buttefly curve\n2.Hypotrochoid\n3.Star\n\n'Q' to stop\n").strip().lower()
        msg = String()
        msg.data = shape
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {shape}")

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
