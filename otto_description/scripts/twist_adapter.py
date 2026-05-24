#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

"""
=============================================================================
File: twist_adapter.py
Role: The Translator (Message Format Adapter)

Description:
This lightweight bridge node solves a common ROS 2 format mismatch between 
standard velocity tools and strict hardware controllers. 

What it does:
1. Listens: Subscribes to standard 'Twist' messages (raw velocity commands 
   without time data) coming out of the priority multiplexer ('/cmd_vel_out').
2. Wraps & Stamps: Takes those raw speed numbers and wraps them in a 
   'TwistStamped' envelope. It attaches the exact current ROS clock time and 
   the physical frame reference ('base_footprint').
3. Forwards: Sends the newly stamped messages directly into the 
   'diff_drive_controller', which requires this timing data to safely and 
   accurately execute wheel rotations.
=============================================================================
"""

class TwistAdapter(Node):
    def __init__(self):
        super().__init__('twist_adapter')
        #  Listen to the Unstamped output from the Multiplexer
        self.sub = self.create_subscription(Twist, '/cmd_vel_out', self.listener_callback, 10)
        
        #  Publish to the Stamped input of the Wheel Controller
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.get_logger().info("OTTO Twist Adapter is running...")

    def listener_callback(self, msg):
        #  Take the math, wrap it in a timestamp, and send it
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