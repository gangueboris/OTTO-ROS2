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

class OttoTeleop(Node):
    def __init__(self):
        super().__init__('otto_teleop')

        # One shared group for all callbacks that touch shared state (gait, step_index, is_walking)
        # MutuallyExclusive guarantees command_callback and timer_callback never overlap
        self.main_cb_group = MutuallyExclusiveCallbackGroup()

        self.publisher_pos_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.publisher_vel_ = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)
        self.subscriber_otto_cmd = self.create_subscription(String, '/otto_command', self.command_callback, 10, callback_group=self.main_cb_group)

        self.joint_names = ['left_hip_joint', 'left_foot_joint', 'right_hip_joint', 'right_foot_joint']

        self.switcher = SwitchMode()
        self.step_index = 0
        self.is_walking = False
        self.timer = None
        self.current_mode = 'walk'

        # Guard flag to ignore incoming commands while a mode switch is in progress
        self.is_switching = False

        # Repeating velocity publisher — keeps diff_drive_controller fed past its cmd_vel_timeout
        self.vel_timer = None
        self.current_vel = (0.0, 0.0) # (linear_x, angular_z)

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
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],  self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],  self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],  self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,   0.0],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,         PIVOT], self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,         PIVOT], self.SPEED_CENTER)
            ],
            'backward': [
                # Reverse the pivots: Left is +, Right is -
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,         PIVOT, 0.0,         0.0],  self.SPEED_LEAN),
                ([ 0.0,         PIVOT, 0.0,         0.0],  self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,   0.0],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT], self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT], self.SPEED_CENTER)
            ],
            'left': [
                # Tank turn left — both feet pivot in the same direction (negative)
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],  self.SPEED_LEAN),
                ([ 0.0,        -PIVOT, 0.0,         0.0],  self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,   0.0],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT], self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,        -PIVOT], self.SPEED_CENTER)
            ],
            'right': [
                # Tank turn right — both feet pivot in the same direction (positive)
                ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN),
                ([ 0.0,         PIVOT, 0.0,         0.0],  self.SPEED_LEAN),
                ([ 0.0,         PIVOT, 0.0,         0.0],  self.SPEED_CENTER),
                ([-LEAN_OUTER,  0.0, -LEAN_INNER,   0.0],  self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,         PIVOT], self.SPEED_LEAN),
                ([ 0.0,         0.0,  0.0,         PIVOT], self.SPEED_CENTER)
            ]
        }

        # Default to forward
        self.active_gait = self.gaits['forward']

    def _schedule_next(self):
        # Timerkeeper, set an alarm clock for the robot next move.
        if not self.is_walking:
            return
        _, duration = self.active_gait[self.step_index]
        # Timer uses the same callback group so it can't overlap with command_callback
        self.timer = self.create_timer(duration, self.timer_callback, callback_group=self.main_cb_group)

    def timer_callback(self):
        if self.timer:
            self.timer.cancel()   # Reset the timer if active
            self.destroy_timer(self.timer)
            self.timer = None
        if not self.is_walking:
            return

        positions, duration = self.active_gait[self.step_index]
        self.send_pose(positions, duration * 0.9)

        # Always reset step_index bounds against the currently active gait length
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

    def _start_vel_loop(self, linear_x, angular_z):
        # Store the target velocity
        self.current_vel = (linear_x, angular_z)

        # Cancel any existing loop before starting a new one
        if self.vel_timer:
            self.vel_timer.cancel()
            self.destroy_timer(self.vel_timer)

        # Publish at 10Hz — well within any typical cmd_vel_timeout
        self.vel_timer = self.create_timer(0.1, self._vel_loop_callback, callback_group=self.main_cb_group)
        self.get_logger().info(f'Velocity loop started: linear={linear_x}, angular={angular_z}')

    def _vel_loop_callback(self):
        # Continuously feed the diff_drive_controller so it never times out
        self.send_vel(self.current_vel[0], self.current_vel[1])

    def _stop_vel_loop(self):
        # Cancel the loop and send one explicit zero to flush the controller
        if self.vel_timer:
            self.vel_timer.cancel()
            self.destroy_timer(self.vel_timer)
            self.vel_timer = None
        self.send_vel(0.0, 0.0)

    def roll_forward(self):
        self.get_logger().info('Moving Forward...')
        self._start_vel_loop(self.linear_speed, 0.0)

    def roll_backward(self):
        self.get_logger().info('Moving Backward...')
        self._start_vel_loop(-self.linear_speed, 0.0)

    def turn_left(self):
        self.get_logger().info('Turning Left...')
        self._start_vel_loop(0.0, self.angular_speed)

    def turn_right(self):
        self.get_logger().info('Turning Right...')
        self._start_vel_loop(0.0, -self.angular_speed)

    def stop_rolling(self):
        self.get_logger().info('Stopping...')
        self._stop_vel_loop()


    def _do_mode_switch(self, mode):
        # Run in a plain Python thread so blocking calls (sleep, service wait)
        # don't freeze the ROS callback system
        if mode == 'roll':
            self.switcher.switch_to_roll()
        elif mode == 'walk':
            self.switcher.switch_to_walk()

        # Release the guard once the switch is complete
        self.is_switching = False
        self.get_logger().info(f'Mode switch to [{mode}] complete')


    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received Web Commmand: {command}')

        # Drop commands that arrive while a mode switch is in progress
        if self.is_switching:
            self.get_logger().warn('Mode switch in progress, ignoring command')
            return

        # Handle the switch
        if command.startswith('walk_') or command.startswith('roll_'):
            mode = command.split('_')[0]
            direction = command.split('_')[1]

            # Switch the mode
            if mode != self.current_mode:
                self.is_switching = True

                # Stop walking before switching so the gait loop doesn't keep firing
                self.is_walking = False
                if self.timer:
                    self.timer.cancel()
                    self.destroy_timer(self.timer)
                    self.timer = None

                # Stop rolling before switching so the vel loop doesn't keep firing
                self._stop_vel_loop()

                self.current_mode = mode

                # Run the blocking switch in a background thread, not on the callback thread
                threading.Thread(target=self._do_mode_switch, args=(mode,), daemon=True).start()
                return

            # Handle walking mode
            if self.current_mode == 'walk':
                if direction in self.gaits:
                    self.active_gait = self.gaits[direction]

                    # Always reset step_index on direction change to enter the new gait cleanly
                    self.step_index = 0

                    # Start walking
                    if not self.is_walking:
                        self.is_walking = True
                        self._schedule_next()

            # Handle rolling mode
            elif self.current_mode == 'roll':
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

            if self.current_mode == 'walk':
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