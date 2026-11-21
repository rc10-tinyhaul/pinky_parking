"""
시뮬레이션 전체를 한 번에 띄운다: Gazebo 월드, 로봇들, 로봇마다 Nav2.

로봇 목록(이름, 도메인, 위치)은 config/robots.yaml 에서 읽는다.
Nav2 는 Gazebo 와 로봇이 뜬 뒤(nav_delay 초) 시작한다.

  ros2 launch pinky_parking_bringup sim.launch.py
  ros2 launch pinky_parking_bringup sim.launch.py gui:=false headless:=true camera:=false
  ros2 launch pinky_parking_bringup sim.launch.py nav:=false      (지도 만들기용)
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pinky_parking_bringup.robots import load_robots


def include(package, launch_file, arguments):
    path = Path(get_package_share_directory(package)) / 'launch' / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments={k: str(v) for k, v in arguments.items()}.items())


def setup(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    share = Path(get_package_share_directory('pinky_parking_bringup'))
    world_name, robots = load_robots(arg('robots') or share / 'config/robots.yaml')

    actions = [include('pinky_parking_gazebo', 'world.launch.py', {
        'world_name': arg('world_name') or world_name,
        'gui': arg('gui'), 'headless': arg('headless')})]
    nav_actions = []
    for robot in robots:
        pose = {key: robot[key] for key in ('x', 'y', 'yaw')}
        actions.append(include('pinky_parking_gazebo', 'spawn_robot.launch.py', {
            'name': robot['name'], 'domain_id': robot['domain_id'],
            'vehicle': robot['vehicle'], 'camera': arg('camera'), **pose}))
        nav_actions.append(include('pinky_parking_navigation', 'robot_nav.launch.py', {
            'domain_id': robot['domain_id'], 'map': arg('map'), 'vehicle': robot['vehicle'],
            'localization': arg('localization'), 'max_speed': arg('max_speed'),
            'goal_tolerance': arg('goal_tolerance'), 'max_turn': arg('max_turn'),
            'lookahead': arg('lookahead'), 'fine_amcl': arg('fine_amcl'),
            'pose_from_tf': arg('fine_amcl'), **pose}))
    if arg('nav').lower() == 'true':
        actions.append(TimerAction(period=float(arg('nav_delay')), actions=nav_actions))
    return actions


def generate_launch_description():
    navigation_share = Path(get_package_share_directory('pinky_parking_navigation'))
    default_map = navigation_share / 'maps/factory.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots', default_value='', description='로봇 목록 YAML (기본: config/robots.yaml)'),
        DeclareLaunchArgument(
            'world_name', default_value='', description='월드 파일 이름 (기본: robots.yaml 의 값)'),
        DeclareLaunchArgument('map', default_value=str(default_map), description='지도 YAML'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 창'),
        DeclareLaunchArgument('headless', default_value='false', description='화면 없는 환경용'),
        DeclareLaunchArgument('camera', default_value='true', description='카메라 영상 브리지'),
        DeclareLaunchArgument('nav', default_value='true', description='Nav2 실행'),
        DeclareLaunchArgument(
            'localization', default_value='amcl',
            description='위치 추정: amcl | odom (시작 위치 + 바퀴 기록, 실물 경기장 없을 때 시험용)'),
        DeclareLaunchArgument('nav_delay', default_value='15.0', description='Nav2 시작 전 대기 [s]'),
        DeclareLaunchArgument('max_speed', default_value='', description='직진 속도 상한 [m/s]'),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='', description='도착 허용 거리 [m] (공식 0.25)'),
        DeclareLaunchArgument('max_turn', default_value='', description='회전 속도 상한 [rad/s]'),
        DeclareLaunchArgument('lookahead', default_value='', description='경로 추종 주시 거리 [m]'),
        DeclareLaunchArgument(
            'fine_amcl', default_value='false',
            description='AMCL 을 작은 경기장에 맞게 (실물과 같은 설정으로 시험할 때 true)'),
        OpaqueFunction(function=setup),
    ])
