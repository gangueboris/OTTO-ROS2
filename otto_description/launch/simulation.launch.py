"""
=============================================================================
File: simulation.launch.py
Role: The Foundation (Simulation & Hardware Abstraction Bringup)

Description:
This file is the ground zero for the robot's simulated environment. It bridges 
the gap between the virtual physics world (Gazebo) and the ROS 2 ecosystem.

What it does:
1. Blueprint: Parses the robot's physical description (URDF/XACRO) and 
   publishes its state (TF tree).
2. Environment: Boots up the Gazebo Harmonic simulator with the warehouse world.
3. Spawning: Injects the 'otto' robot model into the virtual environment.
4. Actuation: Loads the essential motor controllers (Diff-Drive for the base, 
   Joint Trajectory for other mechanisms) via the controller_manager.
5. Senses: Establishes the 'ros_gz_bridge' to pipe raw simulated sensor data 
   (LiDAR scans, Camera images, and the simulation Clock) out of Gazebo 
   and into ROS 2 so the SLAM and Navigation stacks can use them.
=============================================================================
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_name = 'otto_description'
    urdf_path = 'urdf/otto_main.urdf.xacro'
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'living_room.sdf')

    # Process the URDF
    xacro_file = os.path.join(get_package_share_directory(pkg_name), urdf_path)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True 
        }]
    )

    # Launch Gazebo Harmonic
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        #launch_arguments={'gz_args': '-r empty.sdf', 'on_exit_shutdown': 'true'}.items(),     # -r : run, empty.sdf  : the world to load 
        launch_arguments={'gz_args': f'-r {world_path}', 'on_exit_shutdown': 'true'}.items(),
    )

    # Spawn the Robot in Gazebo
    spawn_entity = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-string', robot_description_raw,
                '-name', 'otto',
                '-x', '0.5',
                '-y', '1',
                '-z', '0.05'                                 # spawn height (5 cm above ground), avoids collision with ground at spawn
            ]
        )]
    )

    # Spawn joint_state_broadcaster
    spawn_broadcaster = TimerAction(
        period=8.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )
    
    # Spawn joint_trajectory_controller
    spawn_trajectory_controller = TimerAction(
        period=8.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    # Spawn diff_drive_controller
    spawn_diff_drive = TimerAction(
            period=8.0,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller', '--controller-manager', '/controller_manager', '--inactive'],
                output='screen'
            )]
        )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        spawn_broadcaster,
        spawn_trajectory_controller,
        spawn_diff_drive,
        bridge,
    ])
