"""
관제 서버(API + 대시보드)를 띄운다. 시뮬레이션(sim.launch.py)과 따로 실행한다.

서버는 robots.yaml 의 로봇마다 그 ROS_DOMAIN_ID 에 직접 붙으므로, 이 launch 의 도메인은 상관없다.

  ros2 launch pinky_parking_control control.launch.py
  ros2 launch pinky_parking_control control.launch.py port:=8081
  ros2 launch pinky_parking_control control.launch.py only:=pinky1     (한 대만 연결)
  브라우저: http://localhost:8080
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    arguments = []
    for key in ('robots', 'port', 'only', 'log_dir', 'room_map'):
        value = LaunchConfiguration(key).perform(context)
        if value:
            arguments += [f'--{key.replace("_", "-")}', value]
    for item in LaunchConfiguration('cameras').perform(context).split(','):
        if item:
            arguments += ['--camera', item]
    for item in LaunchConfiguration('references').perform(context).split(','):
        if item:
            arguments += ['--reference', item]
    return [Node(package='pinky_parking_control', executable='control_server',
                 output='screen', arguments=arguments)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots', default_value='', description='로봇 목록 YAML (기본: bringup 의 robots.yaml)'),
        DeclareLaunchArgument('port', default_value='', description='포트 (기본: control.yaml)'),
        DeclareLaunchArgument(
            'only', default_value='', description='일부 로봇만 연결: 이름을 쉼표로 (예: pinky1)'),
        DeclareLaunchArgument(
            'log_dir', default_value='',
            description='궤적 파일을 남길 폴더 (기본: PINKY_LOG_DIR 또는 ~/pinky_logs)'),
        DeclareLaunchArgument(
            'cameras', default_value='',
            description='카메라 영상 주소: 이름=주소 를 쉼표로 (예: pinky1=http://192.168.0.5:8090/stream)'),
        DeclareLaunchArgument(
            'references', default_value='',
            description='기준 경로 CSV: 이름=파일 을 쉼표로 (예: pinky1=/data/coords_robot_a.csv)'),
        DeclareLaunchArgument(
            'room_map', default_value='',
            description='경기장 화면에 겹쳐 보여 줄 고정벽 지도 YAML (snapshot_map 이 만든 것)'),
        OpaqueFunction(function=setup),
    ])
