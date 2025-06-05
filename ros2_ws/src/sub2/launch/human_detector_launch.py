from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sub2',
            node_executable='human_detector',
            node_name='human_detector'
        ),
        Node(
            package='sub2',
            node_executable='object_estimator',
            node_name='object_estimator'
        )


    ])



