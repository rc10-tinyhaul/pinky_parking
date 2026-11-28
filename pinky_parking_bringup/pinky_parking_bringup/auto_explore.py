"""
지도 만들기용 단순 자동 탐색. 키보드 조종 대신 쓴다.

앞이 비면 직진, 막히면 넓은 쪽으로 회전, 가끔 방향을 틀어 구석도 본다.
Pinky LiDAR 프레임은 로봇 정면 기준 180도 돌아 있다 (로봇 각도 = 스캔 각도 - pi).

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup auto_explore 240     # 240초 동안
  # 실물
  ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup auto_explore 180 --speed 0.08 --turn 0.5 \
      --radius 2.5

기본 속도(직진 0.18 m/s, 회전 0.9 rad/s)는 시뮬레이션용이다. 실물은 느리게 준다: 빠르면 SLAM 지도가 휜다.

Nav2 를 거치지 않고 속도 명령을 직접 내므로 스스로 조심한다.
- 앞을 각도가 아니라 로봇 폭만큼의 통로로 본다 (비스듬한 벽, 모서리도 걸린다).
- 가까워지면 속도를 줄이고, 옆이 가까우면 반대로 튼다.
- 앞쪽 라이다 값이 많이 비면(검은 물체, 유리 등 반사가 없는 것) 막힌 것으로 본다.
- 가라고 했는데 바퀴 기록이 안 움직이면 부딪힌 것으로 보고 물러나서 돈다.
- --radius 를 주면 출발한 자리에서 그 거리 안에서만 다닌다.
- 다른 곳에서 정지 명령(속도 0)이 오면 끝낸다: 관제 화면의 전체 정지(Esc)로 멈출 수 있다.
- 라이다가 끊기면 선다.
라이다 높이(약 12.5 cm) 밖의 물건(전선, 낮은 상자)은 여전히 못 본다. 실물은 사람이 옆에서 지켜본다.
"""
import argparse
import math
import random
import sys
import time

LIDAR_X = -0.017        # 라이다가 로봇 중심보다 뒤에 있는 거리 [m] (pinky_description)
HALF_WIDTH = 0.13       # 앞 통로의 반 폭 [m]: 로봇 반 폭 0.06 + 여유
STOP_DISTANCE = 0.30    # 통로 안 가장 가까운 점이 이보다 가까우면 돈다 [m] (로봇 중심 기준)
SLOW_DISTANCE = 0.60    # 이보다 가까우면 속도를 줄인다 [m]
SIDE_DISTANCE = 0.22    # 옆이 이보다 가까우면 반대로 튼다 [m]
MIN_SPEED_RATIO = 0.4   # 가장 느릴 때의 속도 비율 (너무 느리면 실물 바퀴가 안 돈다)
BLIND_RATIO = 0.4       # 앞 ±25° 의 라이다 값이 이 비율 넘게 비면 막힌 것으로 본다
SCAN_TIMEOUT = 0.5      # 라이다가 이보다 오래 끊기면 선다 [s]
STALL_TIME = 1.2        # 가라고 했는데 이 시간 동안 안 움직이면 부딪힌 것 [s]
STALL_SPEED = 0.01      # 바퀴 기록 속도가 이보다 작으면 안 움직이는 것 [m/s]
BACK_TIME = 1.5         # 부딪힌 뒤 물러나는 시간 [s]


def scan_points(ranges, angle_min, angle_increment, range_min):
    """
    스캔 → (로봇 기준 점 목록 [(x 앞, y 왼쪽)], 앞 ±25° 에서 값이 빈 비율).

    값이 빈 것: inf, nan, range_min 이하.
    """
    points, front, blind = [], 0, 0
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_increment - math.pi           # 로봇 기준 각도
        ahead = abs(math.atan2(math.sin(a), math.cos(a))) < math.radians(25)
        front += ahead
        if not math.isfinite(r) or r <= range_min:
            blind += ahead
            continue
        points.append((LIDAR_X + r * math.cos(a), r * math.sin(a)))
    return points, (blind / front if front else 1.0)


def clearances(points):
    """점 목록 → 앞 통로, 뒤 통로, 왼쪽, 오른쪽의 가장 가까운 거리 [m] (없으면 10)."""
    front = back = left = right = 10.0
    for x, y in points:
        if abs(y) < HALF_WIDTH:
            if x > 0:
                front = min(front, x)
            else:
                back = min(back, -x)
        if -0.05 < x < 0.25:                                      # 로봇 옆 (앞쪽으로 조금 더 본다)
            if y >= HALF_WIDTH:
                left = min(left, y)
            elif y <= -HALF_WIDTH:
                right = min(right, -y)
    return front, back, left, right


def wider_side(points):
    """왼쪽 앞과 오른쪽 앞 중 더 트인 쪽: +1 왼쪽, -1 오른쪽. 정면 통로의 점은 빼고 본다."""
    left = [math.hypot(x, y) for x, y in points if x > -0.05 and y >= HALF_WIDTH]
    right = [math.hypot(x, y) for x, y in points if x > -0.05 and y <= -HALF_WIDTH]
    return 1.0 if min(left, default=10.0) >= min(right, default=10.0) else -1.0


def drive_command(points, blind, speed):
    """
    앞으로 갈 때의 명령: (선속도, 각속도) 또는 막혔으면 None.

    가까울수록 느리게, 옆이 가까우면 반대로 튼다.
    """
    front, _, left, right = clearances(points)
    if front < STOP_DISTANCE or blind > BLIND_RATIO:
        return None
    ratio = (front - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE)
    linear = speed * max(MIN_SPEED_RATIO, min(1.0, ratio))
    angular = 0.0
    if min(left, right) < SIDE_DISTANCE:
        angular = 0.4 if left > right else -0.4
    elif min(left, right) < 0.5:
        angular = 0.2 * max(-1.0, min(1.0, left - right))
    return linear, angular


def main():
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class Explore(Node):
        def __init__(self, speed, turn, radius):
            super().__init__('auto_explore')
            self.speed, self.turn, self.radius = speed, turn, radius
            self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
            self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
            self.create_timer(0.1, self.step)
            self.points, self.blind, self.scan_time = [], 1.0, 0.0
            self.odom, self.start, self.odom_speed = None, None, 0.0
            self.turn_until, self.turn_dir = 0.0, 1.0
            self.back_until = 0.0
            self.next_kick = time.time() + 15
            self.stuck_since = None
            self.own_zeros = 0              # 내가 보낸 정지 명령 수 (돌아온 것과 남의 것을 가린다)
            self.scans, self.bumps = 0, 0
            self.stopped = ''               # 끝낸 이유

        def on_scan(self, scan):
            self.scans += 1
            self.points, self.blind = scan_points(
                scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min)
            self.scan_time = time.time()

        def on_odom(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.odom = (p.x, p.y, yaw)
            self.odom_speed = abs(msg.twist.twist.linear.x)
            if self.start is None:
                self.start = (p.x, p.y)

        def on_cmd(self, msg):
            if msg.linear.x or msg.angular.z:
                return
            if self.own_zeros > 0:
                self.own_zeros -= 1
            else:
                self.stopped = '다른 곳에서 정지 명령이 왔습니다 (전체 정지)'

        def send(self, linear=0.0, angular=0.0):
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(linear), float(angular)
            if not linear and not angular:
                self.own_zeros += 1
            self.pub.publish(cmd)

        def start_turn(self, now, direction, low, high):
            self.turn_dir = direction
            self.turn_until = now + random.uniform(low, high) * 0.9 / self.turn

        def home_direction(self):
            """출발한 자리 쪽으로 돌 방향 (+1 왼쪽, -1 오른쪽), 그쪽을 보고 있으면 0. 범위 안이면 None."""
            if not self.radius or self.odom is None or self.start is None:
                return None
            x, y, yaw = self.odom
            dx, dy = self.start[0] - x, self.start[1] - y
            if math.hypot(dx, dy) < self.radius:
                return None
            error = math.atan2(dy, dx) - yaw
            error = math.atan2(math.sin(error), math.cos(error))
            return 0.0 if abs(error) < 0.4 else math.copysign(1.0, error)

        def step(self):
            now = time.time()
            if self.stopped:
                return
            if now - self.scan_time > SCAN_TIMEOUT:            # 라이다가 없으면 가지 않는다
                self.send()
                return
            _, back, _, _ = clearances(self.points)
            if now < self.back_until:                          # 부딪힌 뒤 물러나는 중
                if back > 0.25:
                    self.send(-0.05)
                else:
                    self.back_until = 0.0
                    self.send(0.0, self.turn * self.turn_dir)
                return
            if now < self.turn_until:
                self.send(0.0, self.turn * self.turn_dir)
                return
            home = self.home_direction()
            if home:                                           # 범위 밖: 출발한 쪽으로 돈다
                self.send(0.0, self.turn * home)
                return
            command = drive_command(self.points, self.blind, self.speed)
            if command is None:                                # 막힘: 트인 쪽으로 돈다
                self.stuck_since = None
                self.start_turn(now, wider_side(self.points), 0.8, 2.0)
                self.send(0.0, self.turn * self.turn_dir)
                return
            if home is None and now > self.next_kick:          # 가끔 방향을 튼다
                self.next_kick = now + random.uniform(10, 20)
                self.start_turn(now, random.choice([-1.0, 1.0]), 1.0, 3.0)
                self.send(0.0, self.turn * self.turn_dir)
                return
            if self.odom is not None and self.odom_speed < STALL_SPEED:
                self.stuck_since = self.stuck_since or now
                if now - self.stuck_since > STALL_TIME:        # 가라고 했는데 안 움직인다: 부딪힘
                    self.bumps += 1
                    self.stuck_since = None
                    self.back_until = now + BACK_TIME
                    self.start_turn(now + BACK_TIME, wider_side(self.points), 1.5, 2.5)
                    self.send(-0.05 if back > 0.25 else 0.0, 0.0 if back > 0.25 else self.turn)
                    return
            else:
                self.stuck_since = None
            self.send(*command)

    parser = argparse.ArgumentParser(description='지도 만들기용 단순 자동 탐색')
    parser.add_argument('duration', nargs='?', type=float, default=240.0, help='탐색 시간 [s]')
    parser.add_argument('--speed', type=float, default=0.18, help='직진 속도 [m/s] (실물은 0.08 정도)')
    parser.add_argument('--turn', type=float, default=0.9, help='회전 속도 [rad/s] (실물은 0.5 정도)')
    parser.add_argument('--radius', type=float, default=0.0,
                        help='출발한 자리에서 이 거리 안에서만 다닌다 [m]. 0 이면 제한 없음')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Explore(args.speed, args.turn, args.radius)
    end = time.time() + args.duration
    try:
        while time.time() < end and not node.stopped:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.stopped = '멈췄습니다 (Ctrl+C)'
    if rclpy.ok():
        for _ in range(3):                   # 끝: 멈춤 명령 (실물 감시 노드 규칙)
            node.send()
            time.sleep(0.05)
    print(f'탐색 끝. {node.stopped or "시간이 다 됐습니다"}. 받은 스캔 {node.scans}개, '
          f'부딪힘 {node.bumps}번')
    if not node.scans:
        print('/scan 이 없습니다. ROS_DOMAIN_ID 를 확인하세요.')
    return 0 if node.scans else 1


if __name__ == '__main__':
    sys.exit(main())
