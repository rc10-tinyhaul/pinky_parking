"""
라이다 SLAM 으로 지도 위치를 잡는다 (slam_toolbox). robot_nav.launch.py 의 localization:=slam 이 띄운다.

미리 만든 지도 없이도, 로봇이 달리면서 주변(벽, 가구)을 라이다로 보고 map -> odom 을 계속 바로잡는다.
바퀴 기록만 쓸 때(odom)처럼 오차가 쌓이지 않는다. 시작 자리는 x, y, yaw (지도 좌표)로 알려 준다:
slam_toolbox 의 지도 프레임(slam)은 켤 때의 odom 프레임과 같으므로, map -> slam 고정 변환으로 잇는다.

- SLAM 이 만드는 지도는 /slam_map 으로 낸다 (/map 은 map_server 의 경기장 지도가 쓴다).
- tf_pose 가 map -> base_footprint 를 amcl_pose 로 낸다 (관제 서버가 읽는 위치).

이 launch 를 별도 'ros2 launch' 프로세스로 띄워야 그 로봇의 ROS_DOMAIN_ID 가 적용된다.
"""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from pinky_parking_navigation.params import SLAM_FRAME
from pinky_parking_navigation.params import slam_params
import yaml


def setup(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    use_sim_time = arg('use_sim_time').lower() == 'true'
    base = Path(get_package_share_directory('pinky_navigation')) / 'params/mapper_params.yaml'
    params = slam_params(yaml.safe_load(base.read_text()))
    handle = tempfile.NamedTemporaryFile('w', prefix='pinky_slam_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, handle, sort_keys=False)
    handle.close()

    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        namespace='', output='screen',
        parameters=[handle.name, {'use_sim_time': use_sim_time, 'use_lifecycle_manager': False}],
        remappings=[('/map', '/slam_map'), ('/map_metadata', '/slam_map_metadata')])
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    pose = Node(package='pinky_parking_navigation', executable='tf_pose', output='screen',
                parameters=[{'use_sim_time': use_sim_time}])
    # map -> slam: 지금 로봇 자리가 (x, y, yaw) 가 되게. slam 프레임은 켤 때의 odom 프레임과 같아서,
    # 실물처럼 odom 이 0 이 아닌 채로 켜도 맞도록 odom_localizer 가 첫 odom 으로 계산한다.
    start = Node(package='pinky_parking_navigation', executable='odom_localizer', output='screen',
                 parameters=[{'x': float(arg('x')), 'y': float(arg('y')), 'yaw': float(arg('yaw')),
                              'odom_frame': SLAM_FRAME, 'publish_pose': False,
                              'use_sim_time': use_sim_time}])
    return [start, slam, configure, activate, pose]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0', description='시작 위치 x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='시작 위치 y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='시작 방향 [rad]'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='시뮬레이션 시계'),
        OpaqueFunction(function=setup),
    ])
