#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController        # it is a service
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist


class SwitchMode(Node):
    def __init__(self):
        super().__init__('Mode_switcher')
        # Client creation to talk to controller_manager
        self.client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        # Publisher to move the robot during switching
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.vel_pub = self.create_publisher(Twist, '/teleop_cmd_vel', 10)

        # Wait for the controller to run
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the controller_manager service...')


    def ros_sleep(self, seconds):
        # Wait using real time — safe for non-realtime transitions outside of ROS callbacks
        time.sleep(seconds)


    def call_switch(self, activate, deactivate):
        # Helper function to cleanly switch
        req = SwitchController.Request()
         
        # STRICT means it will fail if it can't perfectly stop/start the hardware | BEST_EFFORT will safely ignore the "Controller is already active" warning
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate

        # Send the request
        future = self.client.call_async(req)
    
        # Wait for result with timeout, without blocking the main executor
        timeout = 5.0
        start = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start > timeout:
                self.get_logger().error('Switch service timed out!')
                return False
        
        return future.result().ok


    def move_hips(self, left_angle, right_angle):
        msg = JointTrajectory()
        # FIX: joint_names order now matches otto_teleop.py (left_hip, left_foot, right_hip, right_foot)
        msg.joint_names = ['left_hip_joint', 'left_foot_joint', 'right_hip_joint', 'right_foot_joint']
        point = JointTrajectoryPoint()

        # Apply the target angles to the hips, feet stay flat
        point.positions = [left_angle, 0.0, right_angle, 0.0]

        # Set a time to bend
        point.time_from_start = Duration(sec=0, nanosec=500_000_000) # 0.5s
        msg.points.append(point)
        self.traj_pub.publish(msg)

        # Wait for the trajectory to complete - ss confirmed working with Gazebo sim lag
        self.ros_sleep(5)
    

    def stop_rolling(self):
        # Send velocity (0,0) and wait
        msg = Twist()
        self.vel_pub.publish(msg)
        self.get_logger().info('Stopping roll...')

    
    def switch_to_roll(self):
        self.get_logger().info("--- Switching to roll ---")

        # Wake up the trajectory controller to move the hips
        self.call_switch(['joint_trajectory_controller'], ['diff_drive_controller'])
        self.ros_sleep(0.5)
        
        # Move to position (0, 0) first to guarantee a clean starting posture
        self.move_hips(0.0, 0.0)
        
        # Bend the hips to 90 degrees (left forward, right backward to tuck into wheel posture)
        self.move_hips(1.5708, -1.5708)

        # Set up the brain to diff_drive
        if self.call_switch(['diff_drive_controller'], ['joint_trajectory_controller']):
            self.get_logger().info("== Ready to drive ==")
        else:
            self.get_logger().error("Failed to activate diff_drive!")


    def switch_to_walk(self):
        self.get_logger().info("--- Switching to walk ---")
        
        # Stop the wheels before switching controllers
        self.stop_rolling()
        self.ros_sleep(0.2)

        # Set up the brain to joint_trajectory
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

