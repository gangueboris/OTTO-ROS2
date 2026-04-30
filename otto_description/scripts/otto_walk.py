#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController

class OttoWebTeleop(Node):
    def __init__(self):
        super().__init__('otto_web_teleop')
        
        # --- Publishers & Subscribers ---
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.subscription = self.create_subscription(String, '/otto_command', self.command_callback, 10)
        
        # --- Service Client to Hot-Swap Controllers ---
        self.switch_srv = self.create_client(SwitchController, '/controller_manager/switch_controller')
            
        self.joint_names = ['left_hip_joint', 'left_foot_joint', 'right_hip_joint', 'right_foot_joint']
        
        # State Variables
        self.current_mode = 'walk' # Starts in walk mode
        self.is_walking = False
        self.timer = None
        self.step_index = 0
        
        # --- Kinematics ---
        self.SPEED_LEAN   = 0.8   
        self.SPEED_CENTER = 0.4   
        LEAN_OUTER = 0.65   
        LEAN_INNER = 0.65   
        PIVOT      = 0.52   

        self.gaits = {
            'forward': [
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        +PIVOT],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        +PIVOT],  self.SPEED_CENTER)
            ],
            'left': [
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT],  self.SPEED_CENTER)
            ],
            'right': [
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,        +PIVOT, 0.0,         0.0],   self.SPEED_LEAN),
                ([ 0.0,        +PIVOT, 0.0,         0.0],   self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        +PIVOT],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        +PIVOT],  self.SPEED_CENTER)
            ],
            'backward': [
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,        +PIVOT, 0.0,         0.0],   self.SPEED_LEAN),
                ([ 0.0,        +PIVOT, 0.0,         0.0],   self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT],  self.SPEED_CENTER)
            ]
        }
        self.active_gait = self.gaits['forward']

    def command_callback(self, msg):
        command = msg.data
        
        # 1. Handle Controller Mode Switching
        if command == 'stop' and self.current_mode == 'roll':
            # Stop the wheels
            self.send_velocity(0.0, 0.0)
            return

        if command == 'stop':
            self.stop_walking()
            return

        mode, action = command.split('_') # e.g. "walk_forward" -> "walk", "forward"
        
        if mode != self.current_mode:
            self.switch_mode(mode)
            
        # 2. Execute Action Based on Current Mode
        if self.current_mode == 'walk':
            if action in self.gaits:
                self.active_gait = self.gaits[action]
                if not self.is_walking:
                    self.is_walking = True
                    self.step_index = 0
                    self._schedule_next()
                    
        elif self.current_mode == 'roll':
            self.stop_walking()
            # Transform action into Velocity (Speed)
            linear = 0.0
            angular = 0.0
            if action == 'forward': linear = 0.5
            elif action == 'backward': linear = -0.5
            elif action == 'left': angular = 2.0
            elif action == 'right': angular = -2.0
            
            self.send_velocity(linear, angular)

    def switch_mode(self, new_mode):
        self.get_logger().info(f'Switching mode to: {new_mode}')
        self.current_mode = new_mode
        self.stop_walking()
        self.send_velocity(0.0, 0.0)
        
        req = SwitchController.Request()
        req.strictness = SwitchController.Request.STRICT
        
        # Transform the robot!
        if new_mode == 'roll':
            self.send_pose([1.57, 0.0, -1.57, 0.0], 0.5) # Fold hips into wheel position
            # Wait 0.6 seconds for the animation to finish before hot-swapping
            self.create_timer(0.6, lambda: self._execute_swap(['diff_drive_controller'], ['joint_trajectory_controller']))
            
        elif new_mode == 'walk':
            # Hot-swap first, then stand up
            self._execute_swap(['joint_trajectory_controller'], ['diff_drive_controller'])
            self.create_timer(0.1, lambda: self.send_pose([0.0, 0.0, 0.0, 0.0], 0.5))

    def _execute_swap(self, activate_list, deactivate_list):
        req = SwitchController.Request()
        req.start_controllers = activate_list
        req.stop_controllers = deactivate_list
        req.strictness = SwitchController.Request.STRICT
        
        future = self.switch_srv.call_async(req)
        self.get_logger().info(f'Hot-swapped Brain: ON={activate_list[0]} OFF={deactivate_list[0]}')

    # --- Walk & Roll Helpers ---
    def stop_walking(self):
        self.is_walking = False
        if self.timer: self.timer.cancel()
        if self.current_mode == 'walk': self.send_pose([0.0, 0.0, 0.0, 0.0], 0.5)

    def _schedule_next(self):
        if not self.is_walking: return
        _, duration = self.active_gait[self.step_index]
        self.timer = self.create_timer(duration, self.timer_callback)

    def timer_callback(self):
        if self.timer: self.timer.cancel()
        if not self.is_walking: return
        positions, duration = self.active_gait[self.step_index]
        self.send_pose(positions, duration * 0.9)
        self.step_index = (self.step_index + 1) % len(self.active_gait)
        self._schedule_next()

    def send_pose(self, positions, time_sec):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
        msg.points.append(point)
        self.traj_pub.publish(msg)
        
    def send_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = OttoWebTeleop()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()