"""
실행 중인 Gazebo 월드에 Pinky Pro 한 대를 띄운다.

로봇의 ROS 쪽 노드(robot_state_publisher, 브리지)는 domain_id 의 ROS_DOMAIN_ID 에서 실행한다.
그 도메인에는 로봇 한 대일 때와 같은 이름의 토픽이 나온다: /scan, /odom, /cmd_vel, /tf.

  ros2 launch pinky_parking_gazebo spawn_robot.launch.py name:=pinky1 domain_id:=11
"""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pinky_parking_gazebo.sim import bridge_entries
from pinky_parking_gazebo.sim import sim_urdf
import xacro
import yaml


def write_temp(text, prefix, suffix):
    handle = tempfile.NamedTemporaryFile('w', prefix=prefix, suffix=suffix, delete=False)
    handle.write(text)
    handle.close()
    return handle.name


def setup(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)

    name = arg('name')
    env = {'ROS_DOMAIN_ID': arg('domain_id')}
    camera = arg('camera').lower() == 'true'
    description_share = Path(get_package_share_directory('pinky_description'))

    # 공식 upload_robot.launch.py 와 같은 xacro 인자. Gazebo 안에서 구분하려고 namespace 를 주고,
    # 도메인 쪽 프레임은 접두어 없이 나오게 고친다 (sim.sim_urdf).
    urdf = sim_urdf(xacro.process_file(
        str(description_share / 'urdf/robot.urdf.xacro'),
        mappings={'namespace': f'{name}/', 'is_sim': 'True',
                  'cam_tilt_deg': arg('cam_tilt_deg')}).toxml(), name)
    bridge_file = write_temp(
        yaml.safe_dump(bridge_entries(name, camera), sort_keys=False),
        f'pinky_{name}_bridge_', '.yaml')

    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', additional_env=env,
             parameters=[{'robot_description': urdf, 'ignore_timestamp': False,
                          'use_sim_time': True}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen', additional_env=env,
             parameters=[{'source_list': ['joint_states'], 'rate': 20.0,
                          'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', name=f'create_{name}', output='screen',
             arguments=['-name', name, '-file', write_temp(urdf, f'pinky_{name}_', '.urdf'),
                        '-x', arg('x'), '-y', arg('y'), '-z', '0.1', '-Y', arg('yaw')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='parameter_bridge',
             output='screen', additional_env=env,
             parameters=[{'config_file': bridge_file, 'use_sim_time': True}]),
    ]
    if camera:
        # image_transport 압축 토픽(…/compressed 등)도 접두어 없이
        suffixes = ('', '/compressed', '/compressedDepth', '/theora', '/zstd')
        actions.append(Node(
            package='ros_gz_image', executable='image_bridge', name='image_bridge',
            output='screen', additional_env=env,
            arguments=[f'{name}/camera/image_raw'],
            remappings=[(f'{name}/camera/image_raw{s}', f'camera/image_raw{s}')
                        for s in suffixes],
            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', description='로봇 이름. Gazebo 토픽 접두어로 쓴다'),
        DeclareLaunchArgument('domain_id', description='이 로봇의 ROS_DOMAIN_ID'),
        DeclareLaunchArgument('x', default_value='0.0', description='스폰 위치 x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='스폰 위치 y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='스폰 방향 [rad]'),
        DeclareLaunchArgument('camera', default_value='true', description='카메라 영상 브리지'),
        DeclareLaunchArgument('cam_tilt_deg', default_value='8', description='카메라 기울기 [deg]'),
        OpaqueFunction(function=setup),
    ])
