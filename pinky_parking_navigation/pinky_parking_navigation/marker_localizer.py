"""
카메라가 본 ArUco 마커로 로봇의 지도 위치를 바로잡는다.

로봇의 카메라 서버(scripts/pinky_camera.py)가 내는 /markers 를 읽어 마커마다 로봇 위치를 계산하고,
지금 믿는 위치와 다르면 initialpose 로 알려 준다 (AMCL, odom_localizer 가 받아 고친다).
바퀴가 미끄러지거나 부딪쳐 위치를 잃어도 마커를 다시 보는 순간 돌아온다.

  ros2 run pinky_parking_navigation marker_localizer --ros-args -p camera_url:=http://<로봇 IP>:8090

- 등록: marker_register 토픽(std_msgs/String, JSON {x, y, yaw} = 로봇의 진짜 위치)을 받으면 지금 보이는
  마커들의 지도 위치를 계산해 markers.yaml 에 저장한다. 자로 잴 필요가 없다.
- 상태: marker_status 토픽(JSON)으로 보이는 마커, 등록된 마커, 마지막 보정을 낸다 (관제 화면용).
- 달리는 중(특히 도는 중)에는 영상이 늦게 와서 오차가 커지므로, 천천히 움직일 때만 고친다.
"""
from collections import deque
import json
import math
from pathlib import Path
import threading
import time
import urllib.request

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from pinky_parking_navigation import markers as geo
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
import yaml

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MarkerLocalizer(Node):
    """마커를 보고 위치를 고친다."""

    def __init__(self):
        super().__init__('marker_localizer')
        self.url = self.declare_parameter('camera_url', '').value.rstrip('/')
        self.file = Path(self.declare_parameter('markers_file', '').value).expanduser()
        self.min_fix = float(self.declare_parameter('min_correction', 0.02).value)       # [m]
        self.min_fix_yaw = float(self.declare_parameter('min_correction_yaw', 0.04).value)
        self.max_speed = float(self.declare_parameter('max_linear', 0.25).value)
        self.max_turn = float(self.declare_parameter('max_angular', 0.3).value)
        self.period = float(self.declare_parameter('fix_period', 1.0).value)            # [s]
        self.load()
        self.pose, self.speed = None, (0.0, 0.0)
        self.recent = deque(maxlen=8)        # (시각, {번호: 로봇 기준 마커 위치})
        self.last_fix_time, self.last_fix = 0.0, None
        self.seen, self.error = {}, ''
        self.lock = threading.Lock()
        self.initialpose = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.status = self.create_publisher(String, 'marker_status', LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose, LATCHED)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(String, 'marker_register', self.on_register, 1)
        self.create_timer(0.2, self.tick)
        threading.Thread(target=self.poll, daemon=True).start()
        self.get_logger().info(
            f'마커 위치 보정: {self.url}/markers, 등록된 마커 {sorted(self.table)} ({self.file})')

    # ---- 설정 파일 ------------------------------------------------------
    def load(self):
        self.file_mtime = self.file.stat().st_mtime if self.file.is_file() else 0.0
        data = yaml.safe_load(self.file.read_text(encoding='utf-8')) if self.file.is_file() else {}
        self.config = data or {}
        camera = self.config.get('camera') or {}
        self.size = float(self.config.get('size', 0.05))
        self.sizes = {int(k): float(v) for k, v in (self.config.get('size_by_id') or {}).items()}
        self.focal = float(camera.get('focal_px', 635.0))
        self.center = (float(camera.get('width', 640)) / 2, float(camera.get('height', 480)) / 2)
        self.forward = float(camera.get('forward', 0.05))
        self.max_distance = float(self.config.get('max_distance', 0.6))
        self.table = {int(k): (float(v['x']), float(v['y']), float(v['yaw']))
                      for k, v in (self.config.get('markers') or {}).items()}

    def save(self):
        self.config['markers'] = {
            int(k): {'x': round(v[0], 4), 'y': round(v[1], 4), 'yaw': round(v[2], 4)}
            for k, v in sorted(self.table.items())}
        text = self.file.read_text(encoding='utf-8') if self.file.is_file() else ''
        head = text.split('\nmarkers:')[0] if '\nmarkers:' in text else text    # 설명 주석을 남긴다
        body = yaml.safe_dump({'markers': self.config['markers']}, sort_keys=True)
        self.file.parent.mkdir(parents=True, exist_ok=True)
        self.file.write_text(head.rstrip('\n') + '\n' + body, encoding='utf-8')

    # ---- 입력 ----------------------------------------------------------
    def on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_odom(self, msg):
        self.speed = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def poll(self):
        """카메라 서버에서 마커를 읽는다 (별도 스레드: 와이파이가 느려도 노드가 멈추지 않게)."""
        while rclpy.ok():
            try:
                with urllib.request.urlopen(f'{self.url}/markers', timeout=1.5) as reply:
                    data = json.loads(reply.read())
                found = {}
                for m in data.get('markers', []):
                    result = geo.marker_in_robot(
                        m['corners'], self.sizes.get(int(m['id']), self.size), self.focal,
                        self.center, self.forward)
                    if result and 0.0 < result['distance'] <= self.max_distance:
                        found[int(m['id'])] = result['pose']
                with self.lock:
                    self.recent.append((time.monotonic(), found))
                    self.error = ''
            except Exception as error:  # noqa: B902  카메라가 꺼져 있어도 계속 시도한다
                with self.lock:
                    self.error = str(error)
            time.sleep(0.15)

    def steady(self, now, window=1.0, need=3):
        """최근 window 초 동안 need 번 이상 본 마커 → 로봇 기준 위치의 가운데 값."""
        with self.lock:
            samples = [found for t, found in self.recent if now - t <= window]
        seen = {}
        for found in samples:
            for marker_id, pose in found.items():
                seen.setdefault(marker_id, []).append(pose)
        return {k: geo.median_pose(v) for k, v in seen.items() if len(v) >= need}

    # ---- 등록 ----------------------------------------------------------
    def on_register(self, msg):
        try:
            data = json.loads(msg.data or '{}')
            robot = (float(data['x']), float(data['y']), float(data['yaw']))
        except (ValueError, KeyError, TypeError):
            if self.pose is None:
                self.get_logger().warn('등록 실패: 로봇 위치를 모릅니다')
                return
            robot = self.pose
        seen = self.steady(time.monotonic())
        if not seen:
            self.get_logger().warn('등록 실패: 지금 보이는 마커가 없습니다')
            self.last_fix = {'note': '등록 실패: 보이는 마커 없음', 'time': time.time()}
            return
        self.load()      # 다른 로봇이 그사이 등록한 마커를 지우지 않도록 파일에서 다시 읽고 더한다
        for marker_id, in_robot in seen.items():
            self.table[marker_id] = geo.marker_from_robot(robot, in_robot)
        self.save()
        self.file_mtime = self.file.stat().st_mtime
        ids = ', '.join(str(k) for k in sorted(seen))
        self.last_fix = {'note': f'마커 {ids} 등록', 'time': time.time()}
        self.get_logger().info(f'마커 {ids} 를 등록했습니다 (로봇 위치 {robot[0]:.3f}, {robot[1]:.3f})')

    # ---- 보정 ----------------------------------------------------------
    def reload_if_changed(self):
        """마커 파일이 바뀌었으면 다시 읽는다 (로봇 여러 대가 같은 파일을 쓴다)."""
        try:
            mtime = self.file.stat().st_mtime
        except OSError:
            return
        if mtime != self.file_mtime:
            self.load()
            self.get_logger().info(f'마커 파일이 바뀌어 다시 읽었습니다: {sorted(self.table)}')

    def tick(self):
        now = time.monotonic()
        self.ticks = getattr(self, 'ticks', 0) + 1
        if self.ticks % 25 == 0:        # 5 초마다
            self.reload_if_changed()
        seen = self.steady(now)
        self.seen = seen
        known = {k: v for k, v in seen.items() if k in self.table}
        slow = abs(self.speed[0]) <= self.max_speed and abs(self.speed[1]) <= self.max_turn
        offset = None       # 등록된 마커로 계산한 위치와 지금 믿는 위치의 차이 [m] (보이는 동안)
        if known and self.pose is not None:
            marker_id = min(known, key=lambda k: math.hypot(known[k][0], known[k][1]))   # 가장 가까운 것
            estimate = geo.robot_from_marker(self.table[marker_id], known[marker_id])
            dist, turn = geo.difference(estimate, self.pose)
            offset = round(dist, 3)
        if offset is not None and slow and now - self.last_fix_time >= self.period:
            if dist >= self.min_fix or turn >= self.min_fix_yaw:
                self.send(estimate)
                self.last_fix_time = now
                self.last_fix = {'id': marker_id, 'distance': round(dist, 3),
                                 'yaw': round(turn, 3), 'time': time.time()}
                self.get_logger().info(
                    f'마커 {marker_id}: 위치를 {dist * 100:.1f} cm, {math.degrees(turn):.1f} 도 고쳤습니다')
        with self.lock:
            error = self.error
        self.status.publish(String(data=json.dumps({
            'visible': sorted(seen), 'registered': sorted(self.table), 'last': self.last_fix,
            'error': error, 'offset': offset,
            'nearest': (round(min(math.hypot(p[0], p[1]) for p in seen.values()), 3)
                        if seen else None)})))

    def send(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[0], pose[1]
        msg.pose.pose.orientation.z = math.sin(pose[2] / 2)
        msg.pose.pose.orientation.w = math.cos(pose[2] / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.0004
        msg.pose.covariance[35] = 0.001
        self.initialpose.publish(msg)


def main():
    rclpy.init()
    node = MarkerLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
