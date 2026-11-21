"""
제자리에서 라이다로 주변 고정벽을 찍어 Nav2 지도 파일로 만든다.

달리면서 지도를 그리면(SLAM) 바퀴가 미끄러지거나 사람이 지나갈 때 벽이 번진다. 이 도구는 로봇이
위치를 아는 자리(주차 칸, 마커로 맞춘 직후)에 서 있는 동안 몇 초의 스캔을 모아, 그 자리에서 보이는
벽과 가구를 지도 좌표(경기장 도면 좌표)로 그대로 옮긴다. 움직이지 않으므로 번지지 않는다.

- 여러 스캔에 꾸준히 찍힌 칸만 벽으로 남긴다 (지나가는 사람은 빠진다. 서 있는 사람은 남으니 비켜 선다).
- 로봇에서 벽까지의 사이는 빈 곳, 그 밖은 모름으로 둔다.
- 저장한 지도는 localization:=amcl map:=<이 지도> 로 쓴다 (라이다 + 바퀴 기록으로 위치 추정).

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_navigation snapshot_map --out <폴더>/room
  ROS_DOMAIN_ID=11 ros2 run pinky_parking_navigation snapshot_map --out <폴더>/room \
      --seconds 8 --x 0 --y 0.15 --yaw 1.5708

로봇 위치를 주지 않으면 지금 믿는 위치(amcl_pose)를 쓴다.
"""
import argparse
import math
import sys
import time

FREE, OCCUPIED, UNKNOWN = 254, 0, 205      # PGM 값 (map_server trinary)
LIDAR_X, LIDAR_YAW = -0.017, math.pi       # base_footprint → rplidar_link (공식 URDF: 180° 돌아 있다)


def scan_hits(ranges, angle_min, angle_increment, pose, max_range, min_range=0.12):
    """스캔 한 번 → 지도 좌표의 맞은 점들 [(x, y)]. pose 는 로봇의 (x, y, yaw)."""
    x0, y0, yaw = pose
    lx = x0 + math.cos(yaw) * LIDAR_X
    ly = y0 + math.sin(yaw) * LIDAR_X
    hits = []
    for i, d in enumerate(ranges):
        if not (min_range < d < max_range):          # nan, inf, 너무 가깝거나 먼 값은 버린다
            continue
        a = yaw + LIDAR_YAW + angle_min + i * angle_increment
        hits.append((lx + d * math.cos(a), ly + d * math.sin(a)))
    return (lx, ly), hits


def build_map(scans, pose, resolution=0.03, max_range=6.0, keep=0.4, margin=0.3):
    """
    스캔 여러 번 → (폭, 높이, (원점 x, y), PGM 바이트[윗줄부터]).

    scans: [(ranges, angle_min, angle_increment)]. keep: 전체 스캔 중 이 비율 이상 찍힌 칸만 벽.
    """
    origin_point, counts, free = None, {}, set()
    for ranges, angle_min, angle_increment in scans:
        origin_point, hits = scan_hits(ranges, angle_min, angle_increment, pose, max_range)
        cells = set()
        for hx, hy in hits:
            cells.add((int(math.floor(hx / resolution)), int(math.floor(hy / resolution))))
            steps = int(math.hypot(hx - origin_point[0], hy - origin_point[1]) / resolution)
            ox, oy = origin_point
            for k in range(steps):                   # 로봇에서 벽까지는 빈 곳
                t = k / max(1, steps)
                free.add((int(math.floor((ox + (hx - ox) * t) / resolution)),
                          int(math.floor((oy + (hy - oy) * t) / resolution))))
        for cell in cells:
            counts[cell] = counts.get(cell, 0) + 1
    need = max(1, int(math.ceil(len(scans) * keep)))
    walls = {cell for cell, n in counts.items() if n >= need}
    if not walls:
        raise ValueError('벽으로 남은 칸이 없습니다 (스캔이 비었거나 keep 이 너무 큽니다)')
    known = walls | free
    pad = int(math.ceil(margin / resolution))
    min_i = min(c[0] for c in known) - pad
    max_i = max(c[0] for c in known) + pad
    min_j = min(c[1] for c in known) - pad
    max_j = max(c[1] for c in known) + pad
    width, height = max_i - min_i + 1, max_j - min_j + 1
    pixels = bytearray([UNKNOWN]) * (width * height)
    for i, j in free:
        pixels[(max_j - j) * width + (i - min_i)] = FREE
    for i, j in walls:                               # 벽이 빈 곳보다 우선
        pixels[(max_j - j) * width + (i - min_i)] = OCCUPIED
    return width, height, (min_i * resolution, min_j * resolution), bytes(pixels), len(walls)


def main():
    parser = argparse.ArgumentParser(description='제자리 라이다 스냅샷으로 고정벽 지도 만들기')
    parser.add_argument('--out', required=True, help='저장할 경로 (확장자 없이)')
    parser.add_argument('--seconds', type=float, default=6.0, help='스캔을 모으는 시간 [s]')
    parser.add_argument('--resolution', type=float, default=0.03)
    parser.add_argument('--max-range', type=float, default=6.0, help='이보다 먼 점은 쓰지 않는다 [m]')
    parser.add_argument('--keep', type=float, default=0.4, help='이 비율 이상 찍힌 칸만 벽으로')
    parser.add_argument('--x', type=float, default=None, help='로봇의 지도 위치 (없으면 amcl_pose)')
    parser.add_argument('--y', type=float, default=None)
    parser.add_argument('--yaw', type=float, default=None)
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from pinky_parking_navigation.save_slam_map import write_map
    import rclpy
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node('snapshot_map')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    scans, got = [], {}

    def on_pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        got['pose'] = (p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    node.create_subscription(
        LaserScan, 'scan',
        lambda m: scans.append((list(m.ranges), m.angle_min, m.angle_increment)),
        qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', on_pose, latched)
    deadline = time.monotonic() + args.seconds + 2.0
    start = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if scans and start is None:
            start = time.monotonic()
        if start is not None and time.monotonic() - start >= args.seconds:
            break
    if None not in (args.x, args.y, args.yaw):
        pose = (args.x, args.y, args.yaw)
    elif 'pose' in got:
        pose = got['pose']
    else:
        print('로봇 위치를 모릅니다. --x --y --yaw 로 주거나 위치 추정을 켜세요')
        return 1
    if len(scans) < 5:
        print(f'스캔이 {len(scans)} 번뿐입니다 (/scan 이 오는지 확인하세요)')
        return 1
    width, height, origin, pixels, walls = build_map(
        scans, pose, args.resolution, args.max_range, args.keep)
    path = write_map(args.out, width, height, args.resolution, origin, pixels,
                     note=f'snapshot_map: 로봇 ({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}) 에서 '
                          f'스캔 {len(scans)} 번으로 찍은 고정벽 (좌표 = 경기장 도면 좌표)')
    print(f'저장: {path}  {width} x {height} 칸 ({width * args.resolution:.1f} x '
          f'{height * args.resolution:.1f} m), 벽 {walls} 칸, 스캔 {len(scans)} 번, '
          f'로봇 위치 ({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f}°)')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
