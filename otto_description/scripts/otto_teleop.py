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
        
        self.hip_pub = self.create_publisher(JointTrajectory, '/hip_controller/joint_trajectory', 10)
        self.foot_pub = self.create_publisher(JointTrajectory, '/foot_controller/joint_trajectory', 10)
        self.vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        self.subscription = self.create_subscription(String, '/otto_command', self.command_callback, 10)
        self.switch_srv = self.create_client(SwitchController, '/controller_manager/switch_controller')
            
        self.current_mode = 'walk' 
        self.is_walking = False
        self.timer = None
        self.swap_timer_roll = None
        self.swap_timer_walk = None
        self.step_index = 0
        
        # --- NEW: Continuous Rolling Variables ---
        self.target_linear = 0.0
        self.target_angular = 0.0
        # This timer publishes Twist messages 10 times a second to feed the watchdog!
        self.roll_timer = self.create_timer(0.1, self._roll_loop) 
        
        self.SPEED_LEAN = 0.8; self.SPEED_CENTER = 0.4   
        L_O = 0.65; L_I = 0.65; P = 0.52   

        self.gaits = {
            'forward': [([L_I, 0.0, L_O, 0.0], self.SPEED_LEAN), ([0.0, -P, 0.0, 0.0], self.SPEED_LEAN), ([0.0, -P, 0.0, 0.0], self.SPEED_CENTER), ([-L_O, 0.0, -L_I, 0.0], self.SPEED_LEAN), ([0.0, 0.0, 0.0, +P], self.SPEED_LEAN), ([0.0, 0.0, 0.0, +P], self.SPEED_CENTER)],
            'left':    [([L_I, 0.0, L_O, 0.0], self.SPEED_LEAN), ([0.0, -P, 0.0, 0.0], self.SPEED_LEAN), ([0.0, -P, 0.0, 0.0], self.SPEED_CENTER), ([-L_O, 0.0, -L_I, 0.0], self.SPEED_LEAN), ([0.0, 0.0, 0.0, -P], self.SPEED_LEAN), ([0.0, 0.0, 0.0, -P], self.SPEED_CENTER)],
            'right':   [([L_I, 0.0, L_O, 0.0], self.SPEED_LEAN), ([0.0, +P, 0.0, 0.0], self.SPEED_LEAN), ([0.0, +P, 0.0, 0.0], self.SPEED_CENTER), ([-L_O, 0.0, -L_I, 0.0], self.SPEED_LEAN), ([0.0, 0.0, 0.0, +P], self.SPEED_LEAN), ([0.0, 0.0, 0.0, +P], self.SPEED_CENTER)],
            'backward':[([L_I, 0.0, L_O, 0.0], self.SPEED_LEAN), ([0.0, +P, 0.0, 0.0], self.SPEED_LEAN), ([0.0, +P, 0.0, 0.0], self.SPEED_CENTER), ([-L_O, 0.0, -L_I, 0.0], self.SPEED_LEAN), ([0.0, 0.0, 0.0, -P], self.SPEED_LEAN), ([0.0, 0.0, 0.0, -P], self.SPEED_CENTER)]
        }
        self.active_gait = self.gaits['forward']

    def command_callback(self, msg):
        command = msg.data
        if command == 'stop' and self.current_mode == 'roll':
            self.target_linear = 0.0
            self.target_angular = 0.0
            return
            
        if command == 'stop':
            self.stop_walking()
            return

        mode, action = command.split('_') 
        if mode != self.current_mode:
            self.switch_mode(mode)
            
        if self.current_mode == 'walk':
            if action in self.gaits:
                self.active_gait = self.gaits[action]
                if not self.is_walking:
                    self.is_walking = True; self.step_index = 0; self._schedule_next()
                    
        elif self.current_mode == 'roll':
            self.stop_walking()
            # Changed to target variables, reducing speed to obey 3.14 rad/s physical limits
            self.target_linear = 0.0
            self.target_angular = 0.0
            if action == 'forward': self.target_linear = 0.15
            elif action == 'backward': self.target_linear = -0.15
            elif action == 'left': self.target_angular = 1.0
            elif action == 'right': self.target_angular = -1.0

    # --- NEW: The heartbeat function for the diff drive controller ---
    def _roll_loop(self):
        if self.current_mode == 'roll':
            twist = Twist()
            twist.linear.x = self.target_linear
            twist.angular.z = self.target_angular
            self.vel_pub.publish(twist)

    def switch_mode(self, new_mode):
        self.get_logger().info(f'Switching mode to: {new_mode}')
        self.current_mode = new_mode
        self.stop_walking()
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        if new_mode == 'roll':
            self.send_pose([1.57, 0.0, -1.57, 0.0], 0.5) 
            self.swap_timer_roll = self.create_timer(0.6, self._swap_to_roll)
            
        elif new_mode == 'walk':
            self._execute_swap(['foot_controller'], ['diff_drive_controller'])
            self.swap_timer_walk = self.create_timer(0.1, self._stand_up)

    def _swap_to_roll(self):
        if self.swap_timer_roll: self.swap_timer_roll.cancel()
        self._execute_swap(['diff_drive_controller'], ['foot_controller'])

    def _stand_up(self):
        if self.swap_timer_walk: self.swap_timer_walk.cancel()
        self.send_pose([0.0, 0.0, 0.0, 0.0], 0.5)

    def _execute_swap(self, activate_list, deactivate_list):
        req = SwitchController.Request()
        req.activate_controllers = activate_list
        req.deactivate_controllers = deactivate_list
        req.strictness = SwitchController.Request.STRICT
        self.switch_srv.call_async(req)
        self.get_logger().info(f'Hot-swapped Brain: ON={activate_list[0]} OFF={deactivate_list[0]}')

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
        hip_positions = [positions[0], positions[2]]
        foot_positions = [positions[1], positions[3]]
        dur = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))

        msg_hip = JointTrajectory(); msg_hip.joint_names = ['left_hip_joint', 'right_hip_joint']
        pt_hip = JointTrajectoryPoint(); pt_hip.positions = hip_positions; pt_hip.time_from_start = dur
        msg_hip.points.append(pt_hip); self.hip_pub.publish(msg_hip)

        if self.current_mode == 'walk' or foot_positions == [0.0, 0.0]:
            msg_foot = JointTrajectory(); msg_foot.joint_names = ['left_foot_joint', 'right_foot_joint']
            pt_foot = JointTrajectoryPoint(); pt_foot.positions = foot_positions; pt_foot.time_from_start = dur
            msg_foot.points.append(pt_foot); self.foot_pub.publish(msg_foot)

def main(args=None):
    rclpy.init(args=args)
    node = OttoWebTeleop()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()