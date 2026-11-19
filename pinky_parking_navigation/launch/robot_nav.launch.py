"""
로봇 한 대의 Nav2 를 그 로봇의 ROS_DOMAIN_ID 에서 띄운다.

공식 pinky_navigation/launch/gz_bringup_launch.xml 을 쓰고, 두 가지만 덮어쓴다.
- AMCL 초기 위치: 인자 x, y, yaw (공식 파일의 목록 형식은 Jazzy 에서 읽히지 않는다)
- 트레일러 차량이면 costmap footprint (인자 vehicle, pinky_parking_description 의 차종)
- lifecycle_nodes_nav 에 smoother_server 추가 (공식 기본값에서 빠져 있다)

  ros2 launch pinky_parking_navigation robot_nav.launch.py domain_id:=11
"""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from pinky_parking_description.vehicles import footprint
from pinky_parking_description.vehicles import load_vehicles
from pinky_parking_navigation.params import nav2_params
import yaml

NAV_NODES = ("['controller_server', 'smoother_server', 'planner_server', 'behavior_server', "
             "'bt_navigator', 'waypoint_follower', 'velocity_smoother']")


def setup(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    map_yaml = Path(arg('map')).expanduser()
    if not map_yaml.is_file():
        raise FileNotFoundError(
            f'지도 파일이 없습니다: {map_yaml}\n'
            '먼저 지도를 만들어 저장하세요 (pinky_parking_navigation/README.md 의 "지도 만들기").')
    nav_share = Path(get_package_share_directory('pinky_navigation'))
    base_file = arg('params_file') or str(nav_share / 'params/nav2_params.yaml')
    vehicles = load_vehicles()
    if arg('vehicle') not in vehicles:
        raise ValueError(f"없는 차종입니다: {arg('vehicle')} (있는 차종: {', '.join(vehicles)})")
    params = nav2_params(yaml.safe_load(Path(base_file).read_text()),
                         float(arg('x')), float(arg('y')), float(arg('yaw')),
                         footprint(vehicles[arg('vehicle')]))

    domain_id = arg('domain_id')
    handle = tempfile.NamedTemporaryFile(
        'w', prefix=f'pinky_nav2_domain{domain_id}_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, handle, sort_keys=False)
    handle.close()
    # 별도 'ros2 launch' 프로세스로 띄운다. launch 안에서 환경변수만 바꾸면
    # 포함된 launch 의 노드에는 ROS_DOMAIN_ID 가 적용되지 않는다.
    return [ExecuteProcess(
        name=f'nav2_domain{domain_id}', output='screen',
        additional_env={'ROS_DOMAIN_ID': domain_id},
        cmd=['ros2', 'launch', 'pinky_navigation', 'gz_bringup_launch.xml',
             f'map:={map_yaml}', f'params_file:={handle.name}', 'use_sim_time:=True',
             f'lifecycle_nodes_nav:={NAV_NODES}'])]


def generate_launch_description():
    share = Path(get_package_share_directory('pinky_parking_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('domain_id', description='이 로봇의 ROS_DOMAIN_ID'),
        DeclareLaunchArgument('x', default_value='0.0', description='AMCL 초기 위치 x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='AMCL 초기 위치 y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='AMCL 초기 방향 [rad]'),
        DeclareLaunchArgument(
            'vehicle', default_value='basic',
            description='차종: 트레일러면 footprint 를 늘림 (pinky_parking_description)'),
        DeclareLaunchArgument(
            'map', default_value=str(share / 'maps/factory.yaml'), description='지도 YAML'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='기준 Nav2 파라미터 (기본: pinky_navigation/params/nav2_params.yaml)'),
        OpaqueFunction(function=setup),
    ])
