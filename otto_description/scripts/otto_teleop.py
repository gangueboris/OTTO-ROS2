#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class OttoWebTeleop(Node):
    def __init__(self):
        super().__init__('otto_web_teleop')
        
        # Publisher to motors
        self.publisher_ = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
            
        # Subscriber listening to the web page!
        self.subscription = self.create_subscription(
            String, '/otto_command', self.command_callback, 10)
            
        self.joint_names = ['left_hip_joint', 'left_foot_joint', 'right_hip_joint', 'right_foot_joint']
        self.step_index = 0
        self.is_walking = False
        self.timer = None
        self.SPEED_LEAN   = 0.8   
        self.SPEED_CENTER = 0.4   
        LEAN_OUTER = 0.65  
        LEAN_INNER = 0.65  
        PIVOT      = 0.52  

        self.gait_sequence = [
            # Lean: load left foot, right leg lifts
            ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN,   "Lean left"),

            # Pivot: left hip releases toward 0 while foot twists → foot can move freely
            ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_LEAN,   "Pivot left foot + release hip"),

            # Down: everything neutral, both feet planted
            ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_CENTER, "Down — plant right"),

            # Lean: load right foot, left leg lifts
            ([-LEAN_OUTER,  0.0, -LEAN_INNER,  0.0],   self.SPEED_LEAN,   "Lean right"),

            # Pivot: right hip releases toward 0 while foot twists → foot can move freely
            ([ 0.0,         0.0,  0.0,        +PIVOT],  self.SPEED_LEAN,   "Pivot right foot + release hip"),

            # Down: everything neutral, both feet planted
            ([ 0.0,         0.0,  0.0,        +PIVOT],  self.SPEED_CENTER, "Down — plant left"),
        ]


    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received Web Command: {command}')
        
        if command == 'walk' and not self.is_walking:
            self.is_walking = True
            self.step_index = 0
            self._schedule_next()
            
        elif command == 'stop':
            self.is_walking = False
            if self.timer:
                self.timer.cancel()
            self.stand_straight()
            
        elif command == 'kick':
            self.is_walking = False
            if self.timer:
                self.timer.cancel()
            self.kick()

    def _schedule_next(self):
        if not self.is_walking:
            return
        _, duration, _ = self.gait_sequence[self.step_index]
        self.timer = self.create_timer(duration, self.timer_callback)

    def timer_callback(self):
        if self.timer:
            self.timer.cancel()
        
        if not self.is_walking:
            return

        positions, duration, desc = self.gait_sequence[self.step_index]
        self.send_pose(positions, duration * 0.9)
        self.step_index = (self.step_index + 1) % len(self.gait_sequence)
        self._schedule_next()

    def stand_straight(self):
        self.send_pose([0.0, 0.0, 0.0, 0.0], 0.5)

    def kick(self):
        # Quick kick sequence
        self.send_pose([0.5, 0.0, -0.5, 0.0], 0.5)

    def send_pose(self, positions, time_sec):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
        msg.points.append(point)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OttoWebTeleop()
    print("🌐 Web Teleop Ready! Listening for commands...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()