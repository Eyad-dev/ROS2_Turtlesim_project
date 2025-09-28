#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.srv import TeleportAbsolute
import math

class TurtleCommander(Node):
    def __init__(self):
        super().__init__("turtle_commander")
        self.subscriber = self.create_subscription(
            String, "shape_topic", self.listener_callback, 10
        )
        self.t = 0.0
        
        self.cli = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute",)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for teleport service...")
        
        self.timer = self.create_timer(0.01, self.publish_motion) 
        
        self.get_logger().info("TurtleCommander ready.")

        self.current_shape = None
    
    def teleport(self, x, y, theta = 0.0):
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        self.cli.call_async(req)

    def listener_callback(self, msg: String):
        shape = msg.data
        self.get_logger().info(f"Received shape: {shape}")
        self.current_shape = shape  
        self.t = 0.0

    def publish_motion(self):
        self.t += 0.06

        if self.current_shape == "1":
            t=self.t
            x = math.sin(t) * (math.exp(math.cos(t)) - 2*math.cos(4*t) - (math.sin(t/12))**5)
            y = math.cos(t) * (math.exp(math.cos(t)) - 2*math.cos(4*t) - (math.sin(t/12))**5)
            scale = 0.7  
            x = 5.5 + scale * x
            y = 5.5 + scale * y

            self.teleport(x, y)
        elif self.current_shape == "2":
            t=self.t
            R, r, d = (64.0*0.2), (94.0*0.2), (30.72*0.2) 
            x = (R - r) * math.cos(t) + d * math.cos(((R - r) / r) * t)
            y = (R - r) * math.sin(t) - d * math.sin(((R - r) / r) * t)

            scale = 0.3
            x = 5.5 + scale * x
            y = 5.5 + scale * y
            self.teleport(x, y) 
        elif self.current_shape == "3":
            t = self.t
            k = 5    
            a = 0.5     
            x = (1 + a * math.cos(k * t)) * math.cos(t)
            y = (1 + a * math.cos(k * t)) * math.sin(t)

            scale = 1.5
            x = 3.4 + scale * x
            y = 5.5 + scale * y

            self.teleport(x, y)

        elif self.current_shape == "q":
            pass  

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
