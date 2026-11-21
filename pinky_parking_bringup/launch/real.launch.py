"""
실물 로봇마다 Nav2 를 띄운다 (PC 에서, PINKY_NETWORK=host 컨테이너 안).

로봇 자체(구동, 라이다)는 로봇에서 공식 bringup 과 cmd_vel_watchdog 로 켠다 (docs/real_robot.md).

로봇 목록(이름, 도메인, 시작 위치)은 robots YAML 에서 읽는다. 기본은 경기장 주차 칸에서 시작하는
config/robots_arena.yaml 이고, 경기장이 아직 없으므로 기본 위치 추정은 odom(시작 위치 + 바퀴 기록)이다.

로봇은 자기 도메인 + 100 (pinky1 = 111)에서 켠다. 로봇 시계가 틀려도 되도록 clock_bridge 가
시각을 PC 시계로 고쳐 원래 도메인(11)으로 넘긴다 (clock_bridge.py).

  ros2 launch pinky_parking_bringup real.launch.py                       # 두 대
  ros2 launch pinky_parking_bringup real.launch.py robots:=<yaml> max_speed:=0.15
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pinky_parking_bringup.robots import load_robots


def setup(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    share = Path(get_package_share_directory('pinky_parking_bringup'))
    nav_launch = (Path(get_package_share_directory('pinky_parking_navigation'))
                  / 'launch/robot_nav.launch.py')
    _, robots = load_robots(arg('robots') or share / 'config/robots_arena.yaml')
    only = [n for n in arg('only').split(',') if n]
    cameras = dict(item.split('=', 1) for item in arg('cameras').split(',') if '=' in item)
    actions = []
    for robot in robots:
        if only and robot['name'] not in only:
            continue
        arguments = {
            'domain_id': robot['domain_id'], 'x': robot['x'], 'y': robot['y'],
            'yaw': robot['yaw'], 'map': arg('map'), 'localization': arg('localization'),
            'use_sim_time': 'false', 'max_speed': arg('max_speed'),
            'goal_tolerance': arg('goal_tolerance'), 'max_turn': arg('max_turn'),
            'lookahead': arg('lookahead'),
            'camera_url': cameras.get(robot['name'], ''),
            'fine_amcl': 'true', 'pose_from_tf': 'true'}
        if arg('bridge').lower() == 'true':
            # 로봇은 도메인 + 100 에서 켠다. 중계가 시각을 PC 시계로 고쳐 이 도메인으로 넘긴다
            actions.append(ExecuteProcess(
                name=f'clock_bridge_{robot["name"]}', output='screen',
                cmd=['ros2', 'run', 'pinky_parking_bringup', 'clock_bridge',
                     '--domain', str(robot['domain_id'])]))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_launch)),
            launch_arguments={k: str(v) for k, v in arguments.items()}.items()))
    if not actions:
        raise ValueError(f'띄울 로봇이 없습니다 (only:={arg("only")})')
    return actions


def generate_launch_description():
    maps = Path(get_package_share_directory('pinky_parking_navigation')) / 'maps'
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots', default_value='',
            description='로봇 목록 YAML (기본: config/robots_arena.yaml, 주차 칸에서 시작)'),
        DeclareLaunchArgument(
            'only', default_value='', description='일부만: 이름을 쉼표로 (예: pinky1)'),
        DeclareLaunchArgument(
            'map', default_value=str(maps / 'arena_open.yaml'),
            description='지도 YAML (기본: 벽 없는 경기장 바닥)'),
        DeclareLaunchArgument(
            'localization', default_value='slam',
            description='위치 추정: slam (라이다로 달리면서 맞춤) | odom (바퀴 기록만) | amcl'),
        DeclareLaunchArgument(
            'max_speed', default_value='0.2',
            description='직진 속도 상한 [m/s] (공식 값 0.2). 처음 켜 보는 로봇은 0.1 로 낮춰 시작'),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='0.08',
            description='도착 허용 거리 [m]. 주차 칸(190 x 300 mm)에 들어가도록 공식 0.25 보다 좁힘'),
        DeclareLaunchArgument(
            'max_turn', default_value='0.8',
            description='회전 속도 상한 [rad/s]. 바퀴 미끄러짐(방향 오차)을 줄이려고 낮춤'),
        DeclareLaunchArgument(
            'cameras', default_value='',
            description='로봇 카메라 서버: 이름=주소 를 쉼표로 (예: pinky1=http://192.168.0.5:8090). '
                        '주면 마커(ArUco)로 위치를 바로잡는다'),
        DeclareLaunchArgument(
            'bridge', default_value='true',
            description='시계 보정 중계(clock_bridge). 로봇을 도메인 + 100 에서 켠다. '
                        'false 면 로봇을 같은 도메인에서 켜고 시계가 맞아야 한다'),
        DeclareLaunchArgument(
            'lookahead', default_value='0.2',
            description='경로 추종 주시 거리 [m]. 트랙 반경 0.4 m 를 안쪽으로 파고들지 않게 짧게'),
        OpaqueFunction(function=setup),
    ])
