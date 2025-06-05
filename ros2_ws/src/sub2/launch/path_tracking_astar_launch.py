#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
import rclpy
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # 백엔드랑 HW 통신 MQTT 
        Node(
            package='comm',
            node_executable='mqtt',
            node_name='mqtt',
            output='screen'
        ),
        # 게이트 정보 수신
        Node(
            package='comm',
            node_executable='gate_receive',
            node_name='gate_receive',
            output='screen'
        ),
        Node(
            package='sub1',
            node_executable='perception',
            node_name='perception',
            #output='screen'
        ),
        Node(
            package='sub1',
            node_executable='handcontrol',
            node_name='handcontrol',
            #output='screen'
        ),
        Node(
            package='sub2',
            node_executable='odom',
            node_name='odom',
            #output='screen'
        ),
        Node(
            package='sub2',
            node_executable='load_map',
            node_name='load_map',
            #output='screen'
        ),
        # 자동 goal 퍼블리셔 노드 추가
        Node(
            package='sub2',
            node_executable='auto_goal_publisher',
            node_name='auto_goal_publisher',
            output='screen'
        ),
        Node(
            package='sub2',
            node_executable='a_star',
            node_name='a_star',
            #output='screen'
        ),
        Node(
            package='sub2',
            node_executable='a_star_local_path',
            node_name='a_star_local_path',
            #output='screen'
        ),
        Node(
            package='sub2',
            node_executable='path_tracking',
            node_name='path_tracking',
            #output='screen'
        ),
        
        TimerAction(
            period = 40.0,  # 40초 대기 후 실행
            actions=[
                Node(
                    package='comm',
                    node_executable='photo_send',
                    node_name='photo_send',
                    output='screen'
                )
            ]
        ),
        # TimerAction(
        #     period = 40.0,  # 40초 대기 후 실행
        #     actions=[
        #         Node(
        #             package='comm',
        #             node_executable='status_send',
        #             node_name='status_send',
        #             output='screen'
        #         )
        #     ]
        # ),
    ])

if __name__ == '__main__':
    generate_launch_description()
