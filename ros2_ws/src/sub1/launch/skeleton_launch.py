from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sub1',
            node_executable='publisher_skeleton',
            node_name='publisher_skeleton',
            output='screen'
        ),
        Node(
            package='sub1',
            node_executable='subscriber_skeleton',
            node_name='subscriber_skeleton',
            output='screen'
        )
       
    ])



