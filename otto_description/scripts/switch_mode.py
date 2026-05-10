#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController        # it is a service
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
import time


class SwitchMode(Node):
    def __init__(self):
        super().__init__('Mode_switcher')
        # Client creation to talk to controller_manager
        self.client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        # Publisher to move the robot during switching
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)

        # Wait for the controller to run
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the controller_manager service...')


    def ros_sleep(self, seconds):
        # Wait using the ROS clock to respect simulation time (Gazebo)
        start_time = self.get_clock().now()
        target_duration = rclpy.duration.Duration(seconds=seconds)
        
        # Keep spinning the node until the target time is reached
        while rclpy.ok() and (self.get_clock().now() - start_time) < target_duration:
            time.sleep(0.01) # Safe to use short real-sleeps since the ROS clock updates in the background


    def call_switch(self, activate, deactivate):
        # Helper function to cleanly switch
        req = SwitchController.Request()
         
        # STRICT means it will fail if it can't perfectly stop/start the hardware | BEST_EFFORT will safely ignore the "Controller is already active" warning
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate

        # Send the request
        result = self.client.call(req)
        return result.ok


    def move_hips(self, left_angle, right_angle):
        msg = JointTrajectory()
        msg.joint_names = ['left_hip_joint', 'right_hip_joint', 'left_foot_joint', 'right_foot_joint']
        point = JointTrajectoryPoint()

        # Apply the target angles to the hips
        point.positions = [left_angle, right_angle, 0.0, 0.0]

        # set a time to bend
        point.time_from_start = Duration(sec=0, nanosec=500_000_000) # 0.5s
        msg.points.append(point)
        self.traj_pub.publish(msg)

        # Pause the script to let Gazebo finish moving
        self.ros_sleep(5.0)
    

    def stop_rolling(self):
        # Send velocity (0,0) and wait
        msg = Twist()   # All zeros by default
        self.vel_pub.publish(msg)
        self.get_logger().info('Stopping roll...')

    
    def switch_to_roll(self):
        self.get_logger().info("--- Switching to roll ---")

        # Wake up the trajectory controller to move the hips
        self.call_switch(['joint_trajectory_controller'], ['diff_drive_controller'])
        self.ros_sleep(0.5)
        
        # Move to position (0, 0)
        self.move_hips(0.0, 0.0)
        
        # Bend the hips to 90 degrees
        self.move_hips(1.5708, -1.5708)

        # Set up the brain to diff_drive
        if self.call_switch(['diff_drive_controller'], ['joint_trajectory_controller']):
            self.get_logger().info("== Ready to drive ==")
        else:
            self.get_logger().error("Failed to activate diff_drive!")


    def switch_to_walk(self):
        self.get_logger().info("--- Switching to walk ---")
        
        # Stop the wheels
        self.stop_rolling()

        # Set up the brain to diff_drive
        if self.call_switch(['joint_trajectory_controller'], ['diff_drive_controller']):
            self.ros_sleep(0.5)
            self.move_hips(0.0, 0.0)  # Move to stand up straight
            self.get_logger().info("== Ready to walk ==")
        else:
            self.get_logger().error("Failed to activate joint_trajectory_controller!!")


"""
def main(args=None):
    rclpy.init(args=args)

    # Check if the user passed 'walk' or 'roll' in the terminal
    if len(sys.argv) < 2:
        print("Usage: python3 switch_mode.py [walk | roll]")
        sys.exit(1)
        
    mode = sys.argv[1].lower()
    node = SwitchMode()
    
    if mode == "roll":
        node.switch_to_roll()
    elif mode == "walk":
        node.switch_to_walk()
    else:
        node.get_logger().error("Invalid mode! Use 'walk' or 'roll'")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""