#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='advanced',
        #     node_executable='load_map',
        #     name='load_map'
        # ),
        # 전역 경로 계산
        Node(
            package='sub2',
            node_executable='a_star',
            name='a_star'
        ),
        # 로컬 경로 추출
        Node(
            package='sub2',
            node_executable='a_star_local_path',
            name='a_star_local_path'
        ),
        Node(
            package='sub1',
            node_executable='controller_modified',
            name='controller_modified'
        ),
        Node(
            package='sub1',
            node_executable='handcontrol',
            name='handcontrol'
        ),
        #센서 데이터 처리
        Node(
            package='sub2',
            node_executable='odom',
            name='odom'
        )
    ])
