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

        self.joint_names = [ 'left_hip_joint', 'left_foot_joint', 'right_hip_joint', 'right_foot_joint']

        self.step_index = 0
        self.is_walking = False
        self.timer = None

        # Kinematic variables
        self.SPEED_LEAN = 0.8
        self.SPEED_CENTER = 0.4
        LEAN_OUTER = 0.65
        LEAN_INNER = 0.65
        PIVOT = 0.52

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
        self.publisher_walk_.publish(msg)


    def stand_straight(self):
        self.send_pose([0.0, 0.0, 0.0, 0.0], 0.5)
    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received Web Commmand: {command}')

        # Handle the command
        # Parse walk commands "walk_forward"
        if command.startswith('walk_'):
            direction = command.split('_')[1]

            if direction in self.gaits:
                self.active_gait = self.gaits[direction]

                # Start walking
                if not self.is_walking:
                    self.is_walking = True
                    self.step_index = 0
                    self._schedule_next()



        # Parse roll commands "roll_stop"
        elif command.startswith('roll_'):
            direction = command.split('_')[1]

        # Handle the stop
        elif command == 'stop':
            self.is_walking = False
            if self.timer:
                self.timer.cancel()
            self.stand_straight()


        
        












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