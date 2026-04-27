#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys, select, termios, tty

class OttoTeleop(Node):
    def __init__(self):
        super().__init__('otto_teleop')
        
        # 1. Create the Publisher (The "Voice" speaking to the controller)
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        # We must send commands to all 4 joints in this exact order
        self.joint_names = [
            'left_hip_joint', 
            'left_foot_joint', 
            'right_hip_joint', 
            'right_foot_joint'
        ]

    def publish_pose(self, positions, time_sec):
        """Helper function to quickly package and send a trajectory"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=time_sec, nanosec=0)
        
        msg.points.append(point)
        self.publisher_.publish(msg)


# --- TERMINAL MAGIC ---
# This function reads a single keypress instantly without waiting for 'Enter'
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    # Save the terminal's default settings so we don't break it
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = OttoTeleop()
    
    print("""
    =======================================
    🤖 OTTO KEYBOARD CONTROLLER ACTIVATED 🤖
    =======================================
    Controls:
      [ w ] : Kick Forward
      [ s ] : Kick Backward
      [ space ] : Stand Up Straight
      [ q ] : Quit
    ---------------------------------------
    """)

    try:
        while True:
            key = get_key(settings)
            
            # 2. The Logic Matrix: Map keys to joint angles
            if key == 'w':
                node.get_logger().info('Forward Kick!')
                node.publish_pose([0.5, 0.0, -0.5, 0.0], 1)
                
            elif key == 's':
                node.get_logger().info('Backward Kick!')
                node.publish_pose([-0.5, 0.0, 0.5, 0.0], 1)
                
            elif key == ' ':
                node.get_logger().info('Standing up...')
                node.publish_pose([0.0, 0.0, 0.0, 0.0], 1)
                
            elif key == 'q':
                print("Shutting down...")
                break

    finally:
        # Clean up and restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()