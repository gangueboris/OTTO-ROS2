#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from switch_mode import SwitchMode
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

"""
=============================================================================
File: otto_teleop.py
Role: The Locomotion Engine (Web-to-Robot Translator)

Description:
This script is the central command hub for manual control. It listens to 
simple text commands sent from the web interface and translates them into 
continuous, coordinated physical movements. 

What it does:
1. Command Listener: Subscribes to '/otto_command' to catch inputs like 
   "walk_forward", "stop", or "switch_roll" from the browser.
2. Dual Locomotion: Manages two entirely different ways of moving:
   - Roll Mode: Sends continuous velocity (Twist) loops to the wheels.
   - Walk Mode: Acts as a gait engine, sequencing precise angular positions 
     (JointTrajectories) to the leg servos frame by frame.
3. Live Tuning: Catches "tune_*" commands from web sliders to instantly 
   recalculate walking kinematics (lean, pivot, stride speed) on the fly 
   without needing to restart the node.
4. State Safety: Safely manages the physical transformation between walking 
   and rolling, ensuring all motors halt before the chassis shifts mode.
=============================================================================
"""

class OttoTeleop(Node):
    def __init__(self):
        super().__init__('otto_teleop')

        # One shared group for all callbacks that touch shared state
        self.main_cb_group = MutuallyExclusiveCallbackGroup()

        # Publishers & Subscribers
        self.publisher_pos_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.publisher_vel_ = self.create_publisher(Twist, '/teleop_cmd_vel', 10)
        self.subscriber_otto_cmd = self.create_subscription(String, '/otto_command', self.command_callback, 10, callback_group=self.main_cb_group)

        self.joint_names = ['left_hip_joint', 'left_foot_joint', 'right_hip_joint', 'right_foot_joint']
        self.switcher = SwitchMode()
        
        # State Tracking
        self.current_mode = 'walk'       # Must match default web UI state
        self.current_direction = 'forward' # Track direction to rebuild gaits safely
        self.is_walking = False
        self.is_switching = False        # Guard flag for mode changes
        self.step_index = 0
        self.timer = None

        # Velocity Loop Tracking
        self.vel_timer = None
        self.current_vel = (0.0, 0.0) # (linear_x, angular_z)

        # ---------------------------------------------------------
        # KINEMATIC VARIABLES (Dynamic / Tunable)
        # ---------------------------------------------------------
        # Drive speeds
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 2.0  # rad/s

        # Walk parameters
        self.SPEED_LEAN = 0.4     # seconds/step
        self.SPEED_CENTER = 0.2
        self.LEAN_OUTER = 0.65
        self.LEAN_INNER = 0.65
        self.PIVOT = 0.52

        # Initialize the dynamic dictionary
        self.gaits = {}
        self._update_gaits()
        self.active_gait = self.gaits['forward']

    def _update_gaits(self):
        """
        Rebuilds the physical gait instructions dynamically.
        Called on boot, and every time the web sliders tune a walk variable.
        """
        self.gaits = {
            'forward': [
                ([ self.LEAN_INNER,  0.0,  self.LEAN_OUTER,  0.0],  self.SPEED_LEAN),
                ([ 0.0,         -self.PIVOT, 0.0,         0.0],  self.SPEED_LEAN),
                ([ 0.0,         -self.PIVOT, 0.0,         0.0],  self.SPEED_CENTER),
                ([-self.LEAN_OUTER,  0.0, -self.LEAN_INNER,   0.0],  self.SPEED_LEAN),
                ([ 0.0,          0.0,  0.0,         self.PIVOT], self.SPEED_LEAN),
                ([ 0.0,          0.0,  0.0,         self.PIVOT], self.SPEED_CENTER)
            ],
            'backward': [
                ([ self.LEAN_INNER,  0.0,  self.LEAN_OUTER,  0.0],  self.SPEED_LEAN),
                ([ 0.0,          self.PIVOT, 0.0,         0.0],  self.SPEED_LEAN),
                ([ 0.0,          self.PIVOT, 0.0,         0.0],  self.SPEED_CENTER),
                ([-self.LEAN_OUTER,  0.0, -self.LEAN_INNER,   0.0],  self.SPEED_LEAN),
                ([ 0.0,          0.0,  0.0,        -self.PIVOT], self.SPEED_LEAN),
                ([ 0.0,          0.0,  0.0,        -self.PIVOT], self.SPEED_CENTER)
            ],
            'left': [
                ([ self.LEAN_INNER,  0.0,  self.LEAN_OUTER,  0.0],  self.SPEED_LEAN),
                ([ 0.0,         -self.PIVOT, 0.0,         0.0],  self.SPEED_LEAN),
                ([ 0.0,         -self.PIVOT, 0.0,         0.0],  self.SPEED_CENTER),
                ([-self.LEAN_OUTER,  0.0, -self.LEAN_INNER,   0.0],  self.SPEED_LEAN),
                ([ 0.0,          0.0,  0.0,        -self.PIVOT], self.SPEED_LEAN),
                ([ 0.0,          0.0,  0.0,        -self.PIVOT], self.SPEED_CENTER)
            ],
            'right': [
                ([ self.LEAN_INNER,  0.0,  self.LEAN_OUTER,  0.0],  self.SPEED_LEAN),
                ([ 0.0,          self.PIVOT, 0.0,         0.0],  self.SPEED_LEAN),
                ([ 0.0,          self.PIVOT, 0.0,         0.0],  self.SPEED_CENTER),
                ([-self.LEAN_OUTER,  0.0, -self.LEAN_INNER,   0.0],  self.SPEED_LEAN),
                ([ 0.0,          0.0,  0.0,         self.PIVOT], self.SPEED_LEAN),
                ([ 0.0,          0.0,  0.0,         self.PIVOT], self.SPEED_CENTER)
            ]
        }
        # Keep the active array pointed at the freshly built dictionary
        self.active_gait = self.gaits[self.current_direction]

    # ==========================
    # WALKING 
    # =========================
    def _schedule_next(self):
        if not self.is_walking: return
        _, duration = self.active_gait[self.step_index]
        self.timer = self.create_timer(duration, self.timer_callback, callback_group=self.main_cb_group)

    def timer_callback(self):
        if self.timer:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
            
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
        self.publisher_pos_.publish(msg)

    def stand_straight(self):
        self.send_pose([0.0, 0.0, 0.0, 0.0], 0.5)

    # ==================
    # ROLLING
    # ==================
    def send_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher_vel_.publish(msg)

    def _start_vel_loop(self, linear_x, angular_z):
        self.current_vel = (linear_x, angular_z)
        if self.vel_timer:
            self.vel_timer.cancel()
            self.destroy_timer(self.vel_timer)
        self.vel_timer = self.create_timer(0.1, self._vel_loop_callback, callback_group=self.main_cb_group)

    def _vel_loop_callback(self):
        self.send_vel(self.current_vel[0], self.current_vel[1])

    def _stop_vel_loop(self):
        if self.vel_timer:
            self.vel_timer.cancel()
            self.destroy_timer(self.vel_timer)
            self.vel_timer = None
        self.send_vel(0.0, 0.0)

    # ==========================================
    # SYSTEM CONTROLS
    # ==========================================
    def _do_mode_switch(self, mode):
        # Runs in a thread so Gazebo has time to physically alter the joints
        if mode == 'roll':
            self.switcher.switch_to_roll()
        elif mode == 'walk':
            self.switcher.switch_to_walk()

        self.is_switching = False
        self.get_logger().info(f'[SYS] Mode switch to [{mode.upper()}] complete')

    def stop_all_motors(self):
        self.is_walking = False
        if self.timer:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        self._stop_vel_loop()
        
        if self.current_mode == 'walk':
            self.stand_straight()

    # ==========================================
    # THE WEB LISTENER 
    # ==========================================
    def command_callback(self, msg):
        command = msg.data
        parts = command.split('_')
        prefix = parts[0]

        # LIVE TUNING (Engineering Drawer)
        if prefix == 'tune':
            param = parts[1]
            value = float(parts[2])
            
            if param == 'linear': self.linear_speed = value
            elif param == 'angular': self.angular_speed = value
            elif param == 'step':
                self.SPEED_LEAN = value
                self.SPEED_CENTER = value / 2.0  # Keep proportion
                self._update_gaits()             # Rebuild dictionary immediately
            elif param == 'pivot':
                self.PIVOT = value
                self._update_gaits()             # Rebuild dictionary immediately
                
            self.get_logger().info(f"[TUNING] Updated {param} to {value}")
            return

        # EMERGENCY STOP
        elif command == 'stop':
            self.get_logger().warn('[STOP] stopping all motors.')
            self.stop_all_motors()
            return

        # GUARD CHECK
        if self.is_switching:
            self.get_logger().warn('[SYS] Ignored command. Chassis is currently shifting modes.')
            return

        # MODE SWITCHING
        if prefix == 'switch':
            new_mode = parts[1] # 'walk' or 'roll'
            if new_mode != self.current_mode:
                self.is_switching = True
                self.stop_all_motors() # Halt physics before transforming
                self.current_mode = new_mode
                threading.Thread(target=self._do_mode_switch, args=(new_mode,), daemon=True).start()
            return

        # MOVEMENT COMMANDS
        # Expected format: "walk_forward", "roll_left"
        if len(parts) == 2:
            mode = parts[0]
            direction = parts[1]
            self.current_direction = direction

            if mode == 'walk' and self.current_mode == 'walk':
                if direction in self.gaits:
                    self.active_gait = self.gaits[direction]
                    self.step_index = 0
                    if not self.is_walking:
                        self.is_walking = True
                        self._schedule_next()

            elif mode == 'roll' and self.current_mode == 'roll':
                if direction == 'forward': self._start_vel_loop(self.linear_speed, 0.0)
                elif direction == 'backward': self._start_vel_loop(-self.linear_speed, 0.0)
                elif direction == 'left': self._start_vel_loop(0.0, self.angular_speed)
                elif direction == 'right': self._start_vel_loop(0.0, -self.angular_speed)


def main(args=None):
    rclpy.init(args=args)
    node = OttoTeleop()
    print("\n====================================")
    print(" OTTO Command Center Backend Online")
    print("====================================\n")
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.switcher)

    try:
        executor.spin() 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()