#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistAdapter(Node):
    def __init__(self):
        super().__init__('twist_adapter')
        # 1. Listen to the Unstamped output from the Multiplexer
        self.sub = self.create_subscription(Twist, '/cmd_vel_out', self.listener_callback, 10)
        
        # 2. Publish to the Stamped input of the Wheel Controller
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.get_logger().info("OTTO Twist Adapter is running...")

    def listener_callback(self, msg):
        # 3. Take the math, wrap it in a timestamp, and send it!
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_footprint'
        stamped_msg.twist = msg
        self.pub.publish(stamped_msg)

def main(args=None):
    print("======= Twist Adapter ==========")
    rclpy.init(args=args)
    rclpy.spin(TwistAdapter())

if __name__ == '__main__':
    main()