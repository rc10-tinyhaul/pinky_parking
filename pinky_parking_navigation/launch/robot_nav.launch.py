"""
로봇 한 대의 Nav2 를 그 로봇의 ROS_DOMAIN_ID 에서 띄운다.

공식 pinky_navigation/launch/bringup_launch.xml 을 쓰고, 두 가지만 덮어쓴다.
- AMCL 초기 위치: 인자 x, y, yaw (공식 파일의 목록 형식은 Jazzy 에서 읽히지 않는다)
- 트레일러 차량이면 costmap footprint (인자 vehicle, pinky_parking_description 의 차종)
- lifecycle_nodes_nav 에 smoother_server 추가 (공식 기본값에서 빠져 있다)
공식 실물용(bringup_launch.xml)과 시뮬레이션용(gz_bringup_launch.xml)은 기본값(map,
use_sim_time)만 다르므로, 둘 다 넘기고 실물용 파일 하나로 쓴다. 실물은 use_sim_time:=false.

  ros2 launch pinky_parking_navigation robot_nav.launch.py domain_id:=11
  ros2 launch pinky_parking_navigation robot_nav.launch.py domain_id:=11 use_sim_time:=false \
      map:=<실물 지도>.yaml        (실물 로봇)
  ros2 launch pinky_parking_navigation robot_nav.launch.py domain_id:=11 localization:=odom \
      x:=0.0 y:=0.15 yaw:=1.5708   (AMCL 없이 시작 위치 + 바퀴 기록, odom_localizer.py)
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
    share = Path(get_package_share_directory('pinky_parking_navigation'))
    base_file = arg('params_file') or str(nav_share / 'params/nav2_params.yaml')
    limits = {key: float(arg(key)) if arg(key) else None
              for key in ('max_speed', 'goal_tolerance', 'max_turn', 'lookahead')}
    fine_amcl = arg('fine_amcl').lower() == 'true'
    vehicles = load_vehicles()
    if arg('vehicle') not in vehicles:
        raise ValueError(f"없는 차종입니다: {arg('vehicle')} (있는 차종: {', '.join(vehicles)})")
    params = nav2_params(yaml.safe_load(Path(base_file).read_text()),
                         float(arg('x')), float(arg('y')), float(arg('yaw')),
                         fine_amcl=fine_amcl, footprint=footprint(vehicles[arg('vehicle')]),
                         **limits)

    domain_id = arg('domain_id')
    use_sim_time = 'True' if arg('use_sim_time').lower() == 'true' else 'False'
    handle = tempfile.NamedTemporaryFile(
        'w', prefix=f'pinky_nav2_domain{domain_id}_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, handle, sort_keys=False)
    handle.close()
    localization = arg('localization')
    if localization not in ('amcl', 'odom', 'slam'):
        raise ValueError(f'localization 은 amcl, odom, slam 중 하나입니다: {localization}')
    cmd = ['ros2', 'launch', 'pinky_navigation', 'bringup_launch.xml',
           f'map:={map_yaml}', f'params_file:={handle.name}', f'use_sim_time:={use_sim_time}',
           f'lifecycle_nodes_nav:={NAV_NODES}']
    actions = []
    if localization == 'odom':
        # AMCL 을 켜지 않고(map_server 만), odom_localizer 가 map -> odom 과 amcl_pose 를 낸다
        cmd.append("lifecycle_nodes_local:=['map_server']")
        actions.append(ExecuteProcess(
            name=f'odom_localizer_domain{domain_id}', output='screen',
            additional_env={'ROS_DOMAIN_ID': domain_id},
            cmd=['ros2', 'run', 'pinky_parking_navigation', 'odom_localizer', '--ros-args',
                 '-p', f"x:={float(arg('x'))}", '-p', f"y:={float(arg('y'))}",
                 '-p', f"yaw:={float(arg('yaw'))}", '-p', f'use_sim_time:={use_sim_time}']))
    if localization == 'amcl' and arg('pose_from_tf').lower() == 'true':
        # AMCL 은 로봇이 움직여 위치를 다시 맞출 때만 amcl_pose 를 낸다. 서 있는 동안에도 관제 화면과
        # 마커 보정이 지금 위치를 알도록, TF(map -> base_footprint)를 같은 토픽으로 계속 낸다.
        actions.append(ExecuteProcess(
            name=f'tf_pose_domain{domain_id}', output='screen',
            additional_env={'ROS_DOMAIN_ID': domain_id},
            cmd=['ros2', 'run', 'pinky_parking_navigation', 'tf_pose', '--ros-args',
                 '-p', f'use_sim_time:={use_sim_time}']))
    if localization == 'slam':
        # AMCL 대신 slam_toolbox 가 라이다로 map -> odom 을 계속 바로잡는다 (미리 만든 지도 불필요)
        cmd.append("lifecycle_nodes_local:=['map_server']")
        actions.append(ExecuteProcess(
            name=f'slam_domain{domain_id}', output='screen',
            additional_env={'ROS_DOMAIN_ID': domain_id},
            cmd=['ros2', 'launch', 'pinky_parking_navigation', 'slam_localization.launch.py',
                 f"x:={float(arg('x'))}", f"y:={float(arg('y'))}", f"yaw:={float(arg('yaw'))}",
                 f'use_sim_time:={use_sim_time}']))
    if arg('camera_url'):
        # 카메라가 본 ArUco 마커로 위치를 바로잡는다 (marker_localizer). 설정 파일은 원본을 고치게 한다
        markers_file = arg('markers_file') or str((share / 'config/markers.yaml').resolve())
        actions.append(ExecuteProcess(
            name=f'marker_localizer_domain{domain_id}', output='screen',
            additional_env={'ROS_DOMAIN_ID': domain_id},
            cmd=['ros2', 'run', 'pinky_parking_navigation', 'marker_localizer', '--ros-args',
                 '-p', f"camera_url:={arg('camera_url')}", '-p', f'markers_file:={markers_file}',
                 # 라이다 위치 추정(AMCL)은 스스로 몇 cm 씩 흔들린다. 그만큼은 마커로 덮어쓰지 않는다
                 '-p', f"min_correction:={0.04 if arg('localization') == 'amcl' else 0.02}",
                 '-p', f"min_correction_yaw:={0.07 if arg('localization') == 'amcl' else 0.04}",
                 '-p', f'use_sim_time:={use_sim_time}']))
    # 별도 'ros2 launch' 프로세스로 띄운다. launch 안에서 환경변수만 바꾸면
    # 포함된 launch 의 노드에는 ROS_DOMAIN_ID 가 적용되지 않는다.
    actions.append(ExecuteProcess(
        name=f'nav2_domain{domain_id}', output='screen',
        additional_env={'ROS_DOMAIN_ID': domain_id}, cmd=cmd))
    return actions


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
            'max_speed', default_value='',
            description='직진 속도 상한 [m/s]. 비우면 공식 값 (실물 첫 시험은 0.1 등으로 낮춤)'),
        DeclareLaunchArgument(
            'pose_from_tf', default_value='false',
            description='amcl 일 때 위치를 TF 에서 계속 낸다 (서 있어도 amcl_pose 가 갱신됨)'),
        DeclareLaunchArgument(
            'fine_amcl', default_value='false',
            description='AMCL 을 작은 경기장에 맞게 (자주 맞춤, 지도에 없는 것에 덜 민감). amcl 일 때만'),
        DeclareLaunchArgument(
            'camera_url', default_value='',
            description='로봇 카메라 서버 주소 (예: http://192.168.0.5:8090). 주면 마커로 위치를 바로잡는다'),
        DeclareLaunchArgument(
            'markers_file', default_value='', description='마커 설정 (기본: config/markers.yaml)'),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='',
            description='도착 허용 거리 [m]. 비우면 공식 값 0.25 (경기장 주차·출차는 0.08 정도)'),
        DeclareLaunchArgument(
            'lookahead', default_value='',
            description='경로 추종 주시 거리 [m]. 비우면 공식 값 (경기장 트랙 반경 0.4 m 에는 0.2)'),
        DeclareLaunchArgument(
            'max_turn', default_value='',
            description='회전 속도 상한 [rad/s]. 비우면 공식 값 (odom 위치 추정은 0.8 정도로 낮춤)'),
        DeclareLaunchArgument(
            'localization', default_value='amcl',
            description='위치 추정: amcl (라이다 + 만든 지도) | slam (라이다로 달리면서 맞춤, 지도 불필요) '
                        '| odom (시작 위치 + 바퀴 기록만)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='시뮬레이션 시계(/clock). 실물 로봇은 false'),
        DeclareLaunchArgument(
            'map', default_value=str(share / 'maps/factory.yaml'), description='지도 YAML'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='기준 Nav2 파라미터 (기본: pinky_navigation/params/nav2_params.yaml)'),
        OpaqueFunction(function=setup),
    ])
