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

    # Include Nav2 + AMCL Bringup script
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': '/home/boris/otto_ws/save_map.yaml',
            'params_file': os.path.join(otto_desc_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        twist_mux_node,
        twist_adapter_node,
        nav2_launch
    ])