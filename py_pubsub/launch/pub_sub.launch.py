from launch import LaunchDescription
from launch_ros.actions import Node

TOPIC = 'communication'

def generate_launch_description():
    publisher = Node(
            package='py_pubsub',
            executable='PubNode',
            name='publisher_node',
            parameters=[{
                "topic": TOPIC
            }]
        )
    
    subscriber = Node(
        package='py_pubsub',
        executable= 'SubNode',
        name= 'subscriber_node',
        parameters=[{
            'topic':TOPIC
        }]
    )

    return LaunchDescription([publisher, subscriber])
    
