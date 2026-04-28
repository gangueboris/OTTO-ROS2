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

        # Timing — "Down" phases need less time than lean/pivot
        self.SPEED_LEAN   = 0.8   # seconds — lean & pivot phases
        self.SPEED_CENTER = 0.4   # seconds — return to center (faster)

        # Amplitudes discovered empirically via rqt
        LEAN_OUTER = 0.65   # hip angle for the unloaded (outer) leg
        LEAN_INNER = 0.65   # hip angle for the loaded (inner) leg
        PIVOT      = 0.52   # foot twist angle

        # fmt: off
        # Each entry: ([positions], duration_sec, description)
        # Positions: [left_hip, left_foot, right_hip, right_foot]
        # "-" from your table = carry previous value (handled by keeping last pos)
        # Left step  → left foot twists NEGATIVE
        # Right step → right foot twists POSITIVE  (opposite sign!)
        self.gait_sequence = [
            # ── LEFT STEP ──────────────────────────────────────────────────────
            # Lean: load left foot, right leg lifts
            ([ LEAN_INNER,  0.0,  LEAN_OUTER,  0.0],   self.SPEED_LEAN,   "Lean left"),

            # Pivot: left hip releases toward 0 while foot twists → foot can move freely
            ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_LEAN,   "Pivot left foot + release hip"),

            # Down: everything neutral, both feet planted
            ([ 0.0,        -PIVOT, 0.0,         0.0],   self.SPEED_CENTER, "Down — plant right"),

            # ── RIGHT STEP ─────────────────────────────────────────────────────
            # Lean: load right foot, left leg lifts
            ([-LEAN_OUTER,  0.0, -LEAN_INNER,  0.0],   self.SPEED_LEAN,   "Lean right"),

            # Pivot: right hip releases toward 0 while foot twists → foot can move freely
            ([ 0.0,         0.0,  0.0,        +PIVOT],  self.SPEED_LEAN,   "Pivot right foot + release hip"),

            # Down: everything neutral, both feet planted
            ([ 0.0,         0.0,  0.0,        +PIVOT],  self.SPEED_CENTER, "Down — plant left"),
        ]

       
        # fmt: on

        self.step_index = 0
        # Start timer with first phase duration
        self._schedule_next()

    def _schedule_next(self):
        """Schedule the next step with its specific duration."""
        _, duration, _ = self.gait_sequence[self.step_index]
        self.timer = self.create_timer(duration, self.timer_callback)

    def timer_callback(self):
        # Cancel the one-shot timer
        self.timer.cancel()

        positions, _, desc = self.gait_sequence[self.step_index]

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(positions)

        # Use the current step's duration for smooth interpolation
        _, duration, _ = self.gait_sequence[self.step_index]
        point.time_from_start = Duration(
            sec=int(duration * 0.9),
            nanosec=int((duration * 0.9 % 1) * 1e9)
        )

        msg.points.append(point)
        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Step {self.step_index + 1}/6 [{desc}] → {positions}'
        )

        self.step_index = (self.step_index + 1) % len(self.gait_sequence)
        self._schedule_next()


def main(args=None):
    rclpy.init(args=args)
    node = OttoWalk()
    print("\n🤖 OTTO WALKING — empirical gait active. Ctrl+C to stop.\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()