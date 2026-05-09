#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class OttoTeleop(Node):
    def __init__(self):
        super().__init__('otto_teleop')
        self.publisher_walk_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.subscriber_otto_cmd = self.create_subscription(String, '/otto_command', self.command_callback, 10)


    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received Web Commmand: {command}')

def main(args=None):
    rclpy.init(args=args)
    node = OttoTeleop()
    print("Web Teleop Ready! Listening for joystick commands...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()