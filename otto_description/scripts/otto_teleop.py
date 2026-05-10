#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
from switch_mode import SwitchMode
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class OttoTeleop(Node):
    def __init__(self):
        super().__init__('otto_teleop')
        self.cmd_cb_group = MutuallyExclusiveCallbackGroup()
        self.publisher_pos_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.publisher_vel_ = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)
        self.subscriber_otto_cmd = self.create_subscription(String, '/otto_command', self.command_callback, 10, callback_group=self.cmd_cb_group)
        
        # Create a callback group to give this subscriber its own dedicated thread

        self.joint_names = [ 'left_hip_joint', 'left_foot_joint', 'right_hip_joint', 'right_foot_joint']

        self.switcher = SwitchMode()
        self.step_index = 0
        self.is_walking = False
        self.timer = None
        self.current_mode = 'walk'

        # Kinematic variables 
        self.SPEED_LEAN = 0.8
        self.SPEED_CENTER = 0.4
        LEAN_OUTER = 0.65
        LEAN_INNER = 0.65
        PIVOT = 0.52

        # Driving speed variables
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0 # rad/s


        # Gait dictionary (4 directionals)
        self.gaits = {
            'forward': [
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        PIVOT],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        PIVOT],  self.SPEED_CENTER)
            ],
            'backward': [
                # Reverse the pivots: Left is +, Right is -
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,        PIVOT, 0.0,         0.0],   self.SPEED_LEAN),
                ([ 0.0,        PIVOT, 0.0,         0.0],   self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT],  self.SPEED_CENTER)
            ],
            'left': [
                # Tank turn left: Both pivots are -
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT],  self.SPEED_CENTER)
            ],
            'right': [
                # Tank turn right: Both pivots are +
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,        PIVOT, 0.0,         0.0],   self.SPEED_LEAN),
                ([ 0.0,        PIVOT, 0.0,         0.0],   self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        PIVOT],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        PIVOT],  self.SPEED_CENTER)
            ]
        }

        # Default to forward
        self.active_gait = self.gaits['forward']
    
    def _schedule_next(self):
        # Timerkeeper, set an alarm clock for the robot next move.
        if not self.is_walking:
            return
        _, duration = self.active_gait[self.step_index]
        self.timer = self.create_timer(duration, self.timer_callback)
    
    def timer_callback(self):
        if self.timer:
            self.timer.cancel()   # Reset the timer if active
            self.destroy_timer(self.timer)
            self.timer = None
        if not self.is_walking:
            return

        positions, duration = self.active_gait[self.step_index]
        self.send_pose(positions, duration * 0.9)

        # Move to next step in the currently active gait
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

    # === Rolling ===
    def send_vel(self, linear_x, angular_z):
        # Helper function to create and publish the Twist message
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        self.publisher_vel_.publish(msg)
        self.get_logger().info(f'Publishing: linear={linear_x}, angular={angular_z}')
    
    def roll_forward(self):
        self.get_logger().info('Moving Foward...')
        self.send_vel(self.linear_speed, 0.0)
    
    def roll_backward(self):
        self.get_logger().info('Moving Backward...')
        self.send_vel(-self.linear_speed, 0.0)

    def turn_left(self):
        self.get_logger().info('Turning Left...')
        self.send_vel(0.0, self.angular_speed)

    def turn_right(self):
        self.get_logger().info('Turning Right...')
        self.send_vel(0.0, -self.angular_speed)

    def stop_rolling(self):
        self.get_logger().info('Stopping...')
        self.send_vel(0.0, 0.0)





    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received Web Commmand: {command}')
        # Handle the switch
        if command.startswith('walk_') or command.startswith('roll_'):
            mode = command.split('_')[0]
            direction = command.split('_')[1]
            
            # Switch the command
            if mode != self.current_mode:
                if mode == 'roll':
                    self.switcher.switch_to_roll()
                    self.current_mode = 'roll'
                elif mode == 'walk':
                    self.switcher.switch_to_walk()
                    self.current_mode = 'walk'

            # Handle walking mode
            if self.current_mode == 'walk':
                if direction in self.gaits:
                    self.active_gait = self.gaits[direction]

                    # Start walking
                    if not self.is_walking:
                        self.is_walking = True
                        self.step_index = 0
                        self._schedule_next()

            # Handle rolling mode
            elif self.current_mode == 'roll':
                self.get_logger().info(f'Switched to mode rolling mode')

                if direction == 'forward':
                    self.roll_forward()
                
                elif direction == 'backward':
                    self.roll_backward()
                
                elif direction == 'left':
                    self.turn_left()
                
                elif direction == 'right':
                    self.turn_right()

                    

        # Handle the stop
        elif command == 'stop':
            self.is_walking = False
            if self.timer:
                self.timer.cancel()
                self.destroy_timer(self.timer)
                self.timer = None
            
            if  self.current_mode == 'walk':
                self.stand_straight()
            else:
                self.stop_rolling()

        
        


def main(args=None):
    rclpy.init(args=args)
    node = OttoTeleop()
    print("Web Teleop Ready! Listening for joystick commands...")
    # Add your main teleop node to the executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.switcher)

    try:
        executor.spin() # Spin both nodes in parallel
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()