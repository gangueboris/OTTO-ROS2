from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Specification of the package name and the robot urdf file within the package
    pkg_name = 'otto_description'
    urdf_path = 'urdf/otto_main.urdf.xacro'

    # Process the file with xacro
    xacro_file = os.path.join(get_package_share_directory(pkg_name), urdf_path)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # RSP Node configuration
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )
   
    # Run the node
    return LaunchDescription([node_robot_state_publisher])