"""
Gazebo 에 로봇을 띄우기 위한 URDF 보정과 브리지 설정.

로봇마다 ROS_DOMAIN_ID 가 다르고, 각 도메인 안의 토픽·프레임은 로봇 한 대일 때와 같은 이름이다
(/scan, /odom, /cmd_vel, /tf, odom, base_footprint). Gazebo 는 하나이고 ROS 도메인과 무관하므로,
Gazebo 안에서만 토픽에 '<name>/' 접두어를 붙여 로봇을 구분하고 브리지가 접두어를 떼어 넘긴다.
"""


def sim_urdf(urdf, name):
    """
    xacro(namespace:='<name>/') 결과를 도메인 분리용으로 고친다.

    - <gazebo reference="<name>/..."> → 실제 링크 이름. 공식 URDF 는 링크에는 접두어를 안 붙이고
      reference 에만 붙여서, 그대로 두면 LiDAR·카메라·IMU 와 바퀴 마찰 설정이 사라진다.
    - Gazebo 가 내보내는 프레임 이름(odom, base_footprint, 센서 frame)은 접두어 없이.
    - 오도메트리 TF 는 로봇별 Gazebo 토픽 '<name>/tf' 로 (전역 /tf 하나에 섞이지 않게).
    Gazebo 토픽 이름(<name>/scan, <name>/cmd_vel …)과 조인트 이름은 접두어를 그대로 둔다.
    """
    p = f'{name}/'
    for tag in ('frame_id', 'child_frame_id', 'gz_frame_id'):
        urdf = urdf.replace(f'<{tag}>{p}', f'<{tag}>')
    urdf = urdf.replace(f'<gazebo reference="{p}', '<gazebo reference="')
    return urdf.replace('<tf_topic>/tf</tf_topic>', f'<tf_topic>{p}tf</tf_topic>')


def bridge_entries(name, camera=True):
    """ros_gz_bridge 설정: Gazebo '<name>/x' ↔ 그 로봇 도메인의 '/x'."""
    def entry(gz, ros, ros_type, gz_type, direction='GZ_TO_ROS'):
        return {'ros_topic_name': ros, 'gz_topic_name': gz, 'ros_type_name': ros_type,
                'gz_type_name': gz_type, 'direction': direction}

    entries = [
        entry('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock'),
        entry(f'{name}/tf', '/tf', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V'),
        entry(f'{name}/scan', '/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan'),
        entry(f'{name}/odom', '/odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
        entry(f'{name}/joint_states', '/joint_states',
              'sensor_msgs/msg/JointState', 'gz.msgs.Model'),
        entry(f'{name}/cmd_vel', '/cmd_vel',
              'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ'),
    ]
    if camera:
        entries.append(entry(f'{name}/camera/camera_info', '/camera/camera_info',
                             'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'))
    return entries
