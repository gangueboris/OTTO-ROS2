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

    xacro_file = os.path.join(get_package_share_directory(pkg_name), urdf_path)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf', 'on_exit_shutdown': 'true'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-string', robot_description_raw, '-name', 'otto', '-z', '0.1']
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'
    )

    spawn_jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')
    
    # The 3 Brains!
    hip_spawner = Node(package='controller_manager', executable='spawner', arguments=['hip_controller'], output='screen')
    foot_spawner = Node(package='controller_manager', executable='spawner', arguments=['foot_controller'], output='screen')
    diff_drive_spawner = Node(package="controller_manager", executable="spawner", arguments=["diff_drive_controller", "--inactive"], output='screen')

    return LaunchDescription([
        node_robot_state_publisher, gazebo, spawn_entity, bridge,
        spawn_jsb,
        TimerAction(period=2.0, actions=[hip_spawner]),
        TimerAction(period=4.0, actions=[foot_spawner]),
        TimerAction(period=6.0, actions=[diff_drive_spawner])
    ])