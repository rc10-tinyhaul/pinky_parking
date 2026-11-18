"""
Gazebo 월드를 띄운다 (서버와, gui:=true 면 창).

월드 파일은 이 패키지의 worlds/ 에서 먼저 찾고, 없으면 pinky_gz_sim/worlds 에서 찾는다.

  ros2 launch pinky_parking_gazebo world.launch.py
  ros2 launch pinky_parking_gazebo world.launch.py gui:=false headless:=true
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    share = Path(get_package_share_directory('pinky_parking_gazebo'))
    gz_share = Path(get_package_share_directory('pinky_gz_sim'))
    description_share = Path(get_package_share_directory('pinky_description'))
    gz_sim_launch = PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py'))

    world_name = LaunchConfiguration('world_name').perform(context)
    world = share / 'worlds' / world_name
    if not world.is_file():
        world = gz_share / 'worlds' / world_name
    if not world.is_file():
        raise FileNotFoundError(f'월드 파일이 없습니다: {world_name}')
    # 화면(DISPLAY)이 없는 곳(도커, 원격)에서는 Gazebo 서버가 EGL 로 센서를 그리게 한다
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    server_args = f'-r -s -v3 {"--headless-rendering " if headless else ""}{world}'

    # 공식 launch_sim.launch.xml 과 같은 모델 경로
    resource_path = ':'.join([
        str(description_share.parent), str(gz_share / 'models'),
        str(Path.home() / '.gazebo/models')])
    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        IncludeLaunchDescription(
            gz_sim_launch,
            launch_arguments={'gz_args': server_args, 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            gz_sim_launch,
            launch_arguments={'gz_args': '-g -v3'}.items(),
            condition=IfCondition(LaunchConfiguration('gui'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name', default_value='pinky_factory.world', description='월드 파일 이름'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 창'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='화면 없는 환경용 (--headless-rendering). gui:=false 와 함께 쓴다'),
        OpaqueFunction(function=setup),
    ])
