"""
=============================================================================
File: web_control.launch.py
Role: The Web Interface & Teleoperation Bridge

Description:
This file sets up the communication layer that allows a web browser frontend 
to interact with, control, and monitor the ROS2 backend.

What it does:
1. WebSocket Bridge (rosbridge): Opens port 9091 to translate standard web 
   traffic into ROS2 messages, allowing a web app to publish and subscribe.
2. Video Streamer (web_video_server): Grabs raw ROS2 image topics (like the 
   robot's camera) and converts them into HTTP streams that can be easily 
   embedded in a webpage.
3. Teleop Engine: Boots up a custom 'otto_teleop.py' node, whichtakes input
   from the web UI (like a virtual joystick) and translates it into velocity
   commands for the differential drive base.
=============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Find the rosbridge_server package
    rosbridge_dir = get_package_share_directory('rosbridge_server')

    # Include the rosbridge websocket XML launch file and set the port
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        launch_arguments={'port': '9091'}.items()
    )

    # Web video server
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server'
    )

    # Setup Teleop Node
    teleop_node = Node(
        package='otto_description', 
        executable='otto_teleop.py',
        name='otto_teleop'
    )

    
    return LaunchDescription([
        rosbridge_launch,
        web_video_server_node,
        teleop_node
    ])