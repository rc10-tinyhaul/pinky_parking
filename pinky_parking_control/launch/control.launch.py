"""
관제 서버(API + 대시보드)를 띄운다. 시뮬레이션(sim.launch.py)과 따로 실행한다.

서버는 robots.yaml 의 로봇마다 그 ROS_DOMAIN_ID 에 직접 붙으므로, 이 launch 의 도메인은 상관없다.

  ros2 launch pinky_parking_control control.launch.py
  ros2 launch pinky_parking_control control.launch.py port:=8081
  브라우저: http://localhost:8080
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    arguments = []
    for key in ('robots', 'port'):
        value = LaunchConfiguration(key).perform(context)
        if value:
            arguments += [f'--{key}', value]
    return [Node(package='pinky_parking_control', executable='control_server',
                 output='screen', arguments=arguments)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots', default_value='', description='로봇 목록 YAML (기본: bringup 의 robots.yaml)'),
        DeclareLaunchArgument('port', default_value='', description='포트 (기본: control.yaml)'),
        OpaqueFunction(function=setup),
    ])
