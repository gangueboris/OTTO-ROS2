import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Find the directories for our packages
    otto_desc_dir = get_package_share_directory('otto_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Setup the Twist Mux Node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(otto_desc_dir, 'config', 'twist_mux.yaml')]
    )

    # Setup the custom Python Adapter Node
    twist_adapter_node = Node(
        package='otto_description', 
        executable='twist_adapter.py',
        name='twist_adapter'
    )

    # 1. PURE NAVIGATION (NO AMCL)
    # We swap 'bringup_launch.py' for 'navigation_launch.py' to leave localization to SLAM
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(otto_desc_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    # 2. LIFELONG MAPPING (SLAM TOOLBOX)
    # We launch SLAM in async mode, overriding its default parameters with our localization file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(otto_desc_dir, 'config', 'mapper_params_localization.yaml')
        }.items()
    )

    return LaunchDescription([
        twist_mux_node,
        twist_adapter_node,
        slam_launch,   # Start Map Reading/Writing
        nav2_launch    # Start Path Planning
    ])