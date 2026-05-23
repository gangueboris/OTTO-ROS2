import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Setup your custom Python Teleop Node
    teleop_node = Node(
        package='otto_description', 
        executable='otto_teleop.py',
        name='otto_teleop'
    )

    # Launch them both!
    return LaunchDescription([
        rosbridge_launch,
        web_video_server_node,
        teleop_node
    ])