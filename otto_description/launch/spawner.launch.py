from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_name = 'otto_description'
    urdf_path = 'urdf/otto_main.urdf.xacro'
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'custom_world.sdf')

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
        launch_arguments={'gz_args': f'-r {world_path}', 'on_exit_shutdown': 'true'}.items(),     # -r : run, empty.sdf: the world to load
    )

    # Spawn the Robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_raw,
            '-name', 'otto',
            '-z', '0.1'                    # spawn height (10 cm above ground), avoids collision with ground at spawn
        ]
    )

    # Spawn joint_state_broadcaster
    spawn_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawn diff_drive_controller
    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Spawn joint_trajectory_controller
    spawn_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Remap '/cmd_vel', '/diff_drive_controller/cmd_vel' because rqt_robot_steering publishes on /cmd_vel and diff_drive_controller listens on /diff_drive_controller/cmd_vel
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        spawn_broadcaster,
        spawn_diff_drive,
        spawn_trajectory_controller,
        bridge,
        rqt_robot_steering
    ])
