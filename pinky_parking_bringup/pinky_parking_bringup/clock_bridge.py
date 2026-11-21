"""
시계 보정 중계: 실물 로봇의 시계가 PC 와 달라도 Nav2 를 쓸 수 있게 한다 (PC 에서 실행).

실물 Pinky 는 시계용 배터리가 없어 켤 때마다 시계가 틀릴 수 있고(며칠씩), 인터넷 시간 동기가
막힌 와이파이에서는 맞춰지지 않는다. 그러면 로봇이 낸 /tf, /odom, /scan 의 시각이 PC 의 Nav2
에게는 '아주 오래된 데이터'라서 주행이 되지 않는다. sudo 없이는 로봇 시계를 못 고친다.

그래서 로봇은 따로 둔 도메인(robot_domain, 기본 = 도메인 + 100)에서 켜고, 이 노드가 그 도메인의
토픽을 받아 시각만 PC 시계로 고쳐 원래 도메인으로 다시 낸다. 속도 명령은 반대로 넘긴다.

  로봇 도메인 → PC 도메인 (시각 보정): odom, scan, tf      (그대로): tf_static, battery/voltage, battery/percent
  PC 도메인 → 로봇 도메인: cmd_vel

  ros2 run pinky_parking_bringup clock_bridge --domain 11            # 로봇은 ROS_DOMAIN_ID=111
  ros2 run pinky_parking_bringup clock_bridge --domain 11 --robot-domain 111

로봇의 정지 감시(cmd_vel_watchdog)는 로봇 도메인에서 그대로 돈다: 이 중계나 와이파이가 끊기면 선다.
"""
import argparse
from collections import deque
import sys
import threading
import time

ROBOT_DOMAIN_OFFSET = 100
WINDOW = 150      # 시계 차이를 재는 최근 odom 개수 (30 Hz 에서 약 5초)


class ClockOffset:
    """로봇 시계와 PC 시계의 차이 [ns]. 받은 시각 - 찍힌 시각의 최솟값(= 전송 지연이 가장 짧았을 때)."""

    def __init__(self, window=WINDOW):
        self.samples = deque(maxlen=window)
        self.value = None

    def update(self, received_ns, stamped_ns):
        self.samples.append(received_ns - stamped_ns)
        self.value = min(self.samples)
        return self.value

    def apply(self, stamped_ns):
        return stamped_ns + self.value if self.value is not None else None


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def set_stamp(stamp, ns):
    stamp.sec, stamp.nanosec = divmod(int(ns), 1_000_000_000)


def main():
    parser = argparse.ArgumentParser(description='실물 로봇 토픽을 PC 시계로 고쳐 중계')
    parser.add_argument('--domain', type=int, required=True, help='PC 쪽(Nav2, 관제) 도메인')
    parser.add_argument('--robot-domain', type=int, default=None,
                        help=f'로봇 쪽 도메인 (기본: domain + {ROBOT_DOMAIN_OFFSET})')
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32
    from tf2_msgs.msg import TFMessage
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    robot_domain = args.robot_domain if args.robot_domain is not None \
        else args.domain + ROBOT_DOMAIN_OFFSET
    if robot_domain == args.domain:
        parser.error('로봇 도메인과 PC 도메인이 같으면 안 됩니다')

    latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    robot_ctx, pc_ctx = Context(), Context()
    rclpy.init(context=robot_ctx, domain_id=robot_domain)
    rclpy.init(context=pc_ctx, domain_id=args.domain)
    robot = rclpy.create_node('clock_bridge_robot', context=robot_ctx)
    pc = rclpy.create_node('clock_bridge', context=pc_ctx)
    log = pc.get_logger()

    offset = ClockOffset()
    lock = threading.Lock()
    static = {}                                   # 자식 프레임 → 변환 (tf_static 은 모아서 한 번에)
    counts = {'odom': 0, 'scan': 0, 'cmd_vel': 0}
    pubs = {
        'odom': pc.create_publisher(Odometry, 'odom', 20),
        'scan': pc.create_publisher(LaserScan, 'scan', qos_profile_sensor_data),
        'tf': pc.create_publisher(TFMessage, 'tf', 100),
        'tf_static': pc.create_publisher(TFMessage, 'tf_static', latched),
        'battery': pc.create_publisher(Float32, 'battery/voltage', 1),
        'battery_pct': pc.create_publisher(Float32, 'battery/percent', 1),
        'cmd_vel': robot.create_publisher(Twist, 'cmd_vel', 10),
    }

    def fix(header):
        """시각을 PC 시계로. 아직 차이를 못 쟀으면 False."""
        with lock:
            fixed = offset.apply(stamp_ns(header.stamp))
        if fixed is None:
            return False
        set_stamp(header.stamp, fixed)
        return True

    def on_odom(msg):
        with lock:
            first = offset.value is None
            value = offset.update(time.time_ns(), stamp_ns(msg.header.stamp))
        if first:
            log.info(f'로봇 시계는 PC 보다 {value / 1e9:+.3f} s 다릅니다. 이만큼 고쳐서 중계합니다')
        fix(msg.header)
        counts['odom'] += 1
        pubs['odom'].publish(msg)

    def on_scan(msg):
        if fix(msg.header):
            counts['scan'] += 1
            pubs['scan'].publish(msg)

    def on_tf(msg):
        if all(fix(t.header) for t in msg.transforms):
            pubs['tf'].publish(msg)

    def on_tf_static(msg):
        for t in msg.transforms:
            static[t.child_frame_id] = t
        pubs['tf_static'].publish(TFMessage(transforms=list(static.values())))

    def on_cmd_vel(msg):
        counts['cmd_vel'] += 1
        pubs['cmd_vel'].publish(msg)

    def report():
        if counts['odom'] == 0:
            log.warn(f'로봇 도메인 {robot_domain} 에서 odom 이 오지 않습니다 (로봇 bringup 확인)')
        counts.update(odom=0, scan=0, cmd_vel=0)

    robot.create_subscription(Odometry, 'odom', on_odom, 20)
    robot.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)
    robot.create_subscription(TFMessage, 'tf', on_tf, 100)
    robot.create_subscription(TFMessage, 'tf_static', on_tf_static, latched)
    robot.create_subscription(
        Float32, 'battery/voltage', lambda m: pubs['battery'].publish(m), 1)
    robot.create_subscription(
        Float32, 'battery/percent', lambda m: pubs['battery_pct'].publish(m), 1)
    pc.create_subscription(Twist, 'cmd_vel', on_cmd_vel, 10)
    pc.create_timer(5.0, report)
    log.info(f'시계 보정 중계: 로봇 도메인 {robot_domain} ↔ PC 도메인 {args.domain}')

    executors = []
    for node, ctx in ((robot, robot_ctx), (pc, pc_ctx)):
        executor = SingleThreadedExecutor(context=ctx)
        executor.add_node(node)
        executors.append(executor)
    worker = threading.Thread(target=executors[0].spin, daemon=True)
    worker.start()
    try:
        executors[1].spin()
    except KeyboardInterrupt:
        pass
    finally:
        pubs['cmd_vel'].publish(Twist())          # 끌 때 정지 명령 한 번
        for ctx in (robot_ctx, pc_ctx):
            rclpy.try_shutdown(context=ctx)
    return 0


if __name__ == '__main__':
    sys.exit(main())
