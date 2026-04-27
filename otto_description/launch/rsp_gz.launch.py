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

    # Process the URDF
    xacro_file = os.path.join(get_package_share_directory(pkg_name), urdf_path)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    node_robot_state_publisher = Node(
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
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),                   # -r : run, empty.sdf: the world to load
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

    # Bridge the Simulation Clock to ROS2 # It is taking a message from Gazebo, reformatting it, and publishing it to ROS 2, and vice versa.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # == ROS2_Controller ==

    # Spawn Joint State Broadcaster
    # This reads the joint positions from Gazebo and sends them to ROS
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn the Trajectory Controller
    # This receives our movement commands and sends them to the Gazebo servos
    spawn_jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        spawn_jsb,
        spawn_jtc
    ])
