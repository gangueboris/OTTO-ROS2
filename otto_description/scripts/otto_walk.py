#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class OttoWalk(Node):
    def __init__(self):
        super().__init__('otto_walk')
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        self.joint_names = [
            'left_hip_joint', 
            'left_foot_joint', 
            'right_hip_joint', 
            'right_foot_joint'
        ]
        
       # SPEED: Increased to 0.8 to give gravity time to shift the weight!
        # If he still slips, try 1.0. If he is stable, you can speed it up to 0.6.
        SPEED = 0.8 
        
        # The Continuous 4-Step Gait (Corrected for Straight Walking)
        # Sequence: [left_hip, left_foot, right_hip, right_foot]
        self.gait_sequence = [
            
            # Phase 1: Lean Left 
            # Weight shifts to left foot. Right foot untwists in the air.
            [0.52, 0.0, 0.78, 0.0],
            
            # Phase 2: Turn Left 
            # Left foot twists negative (-0.52) to pivot body forward.
            [0.52, -0.52, 0.78, 0.0],
            
            # Phase 3: Lean Right
            # Weight shifts to right foot. Left foot untwists in the air.
            [-0.78, 0.0, -0.52, 0.0],
            
            # Phase 4: Turn Right 
            # Right foot twists POSITIVE (+0.52) to pivot body forward!
            [-0.78, 0.0, -0.52, 0.52]
        ]
        
        self.step_index = 0
        self.timer = self.create_timer(SPEED, self.timer_callback)
        self.duration_sec = SPEED

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.gait_sequence[self.step_index]
        
        point.time_from_start = Duration(
            sec=int(self.duration_sec), 
            nanosec=int((self.duration_sec % 1) * 1e9)
        ) 
        
        msg.points.append(point)
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Step {self.step_index + 1}: {self.gait_sequence[self.step_index]}')
        
        # Loop back to step 0 after step 3 (0, 1, 2, 3 -> 0, 1, 2, 3)
        self.step_index = (self.step_index + 1) % 4

def main(args=None):
    rclpy.init(args=args)
    node = OttoWalk()
    
    print("\n🤖 OTTO IS WALKING (CUSTOM GAIT)! Press Ctrl+C to stop.\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()