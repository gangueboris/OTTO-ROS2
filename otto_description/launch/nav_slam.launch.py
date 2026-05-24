"""
=============================================================================
File: nav_slam.launch.py
Role: The Autonomy & Mapping Engine (Nav2 + SLAM)

Description:
This file fires up the robot's brain for autonomous movement and mapping. 
It integrates the ROS 2 Navigation Stack (Nav2) with SLAM so the robot can 
build a map of its environment while navigating it.

What it does:
1. Velocity Multiplexer (twist_mux): Manages movement priorities. It ensures 
   that conflicting speed commands (e.g., autonomous Nav2 driving vs. manual 
   web teleoperation) do not crash the robot by prioritizing one over the other.
2. Command Adapter (twist_adapter): Runs a custom script to translate standard 
   velocity commands into the exact format your differential drive controller 
   expects to receive (unstamped -> stamped).
3. Nav2 & SLAM Toolbox: Boots up the official Nav2 bringup but crucially flags 
   'slam: True'. This disables static localization (AMCL) and turns on dynamic 
   mapping (SLAM Toolbox), tuned via your custom 'nav2_params.yaml'.
=============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    otto_desc_dir = get_package_share_directory('otto_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Setup the Twist Mux Node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(otto_desc_dir, 'config', 'twist_mux.yaml')]
    )

    # Setup the custom Adapter Node
    twist_adapter_node = Node(
        package='otto_description', 
        executable='twist_adapter.py',
        name='twist_adapter'
    )

    # Include Nav2 in SLAM Mode
    nav2_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam': 'True',    # Turns on SLAM Toolbox, turns off AMCL
            'params_file': os.path.join(otto_desc_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        twist_mux_node,
        twist_adapter_node,
        nav2_mapping_launch
    ])


