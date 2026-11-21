"""
SLAM 이 만든 지도를 map 좌표(경기장 도면 좌표)로 돌려 Nav2 지도 파일로 저장한다.

localization:=slam 으로 방을 한 바퀴 돈 뒤 실행한다. slam_toolbox 의 지도는 자기 프레임(slam)에
있으므로, map -> slam 고정 변환(= 켤 때의 시작 자리)만큼 돌리고 옮겨서 저장한다. 그러면 저장한
지도의 좌표가 곧 도면 좌표라서, 다음부터 localization:=amcl map:=<이 지도> 로 위치를 잡을 수 있다.

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_navigation save_slam_map --out <폴더>/room
  → <폴더>/room.yaml, room.pgm
"""
import argparse
import math
from pathlib import Path
import sys
import time

FREE, OCCUPIED, UNKNOWN = 254, 0, 205      # PGM 값 (map_server trinary)


def to_map_frame(width, height, resolution, origin, data, transform):
    """
    Slam 프레임의 격자를 map 프레임 격자로 다시 뽑는다 (가장 가까운 칸).

    data: OccupancyGrid 값 (-1 모름, 0 ~ 100), 아랫줄부터. origin: (x, y) 왼쪽 아래 모서리.
    transform: map -> slam (x, y, yaw). 돌려준다: (폭, 높이, (원점 x, y), PGM 바이트[윗줄부터]).
    """
    tx, ty, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = origin
    corners = [(ox, oy), (ox + width * resolution, oy),
               (ox, oy + height * resolution), (ox + width * resolution, oy + height * resolution)]
    xs = [tx + c * x - s * y for x, y in corners]
    ys = [ty + s * x + c * y for x, y in corners]
    new_ox = math.floor(min(xs) / resolution) * resolution
    new_oy = math.floor(min(ys) / resolution) * resolution
    new_w = int(math.ceil((max(xs) - new_ox) / resolution))
    new_h = int(math.ceil((max(ys) - new_oy) / resolution))
    pixels = bytearray()
    for row in range(new_h - 1, -1, -1):                 # PGM 첫 줄 = 맨 위 (y 가장 큼)
        my = new_oy + (row + 0.5) * resolution
        for col in range(new_w):
            mx = new_ox + (col + 0.5) * resolution
            dx, dy = mx - tx, my - ty                    # map → slam (역변환)
            sx, sy = c * dx + s * dy, -s * dx + c * dy
            i, j = int((sx - ox) // resolution), int((sy - oy) // resolution)
            value = data[j * width + i] if 0 <= i < width and 0 <= j < height else -1
            pixels.append(UNKNOWN if value < 0 else OCCUPIED if value > 50 else FREE)
    return new_w, new_h, (new_ox, new_oy), bytes(pixels)


def write_map(out, width, height, resolution, origin, pixels, note=''):
    """<out>.pgm 과 <out>.yaml 을 쓴다."""
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix('.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels)
    out.with_suffix('.yaml').write_text(
        (f'# {note}\n' if note else '')
        + f'image: {out.name}.pgm\nmode: trinary\nresolution: {resolution}\n'
        + f'origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0]\nnegate: 0\n'
        + 'occupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf-8')
    return out.with_suffix('.yaml')


def main():
    parser = argparse.ArgumentParser(description='SLAM 지도를 map 좌표로 저장')
    parser.add_argument('--out', required=True, help='저장할 경로 (확장자 없이)')
    parser.add_argument('--topic', default='/slam_map')
    parser.add_argument('--frame', default='slam', help='SLAM 지도의 프레임')
    parser.add_argument('--wait', type=float, default=10.0, help='지도를 기다리는 시간 [s]')
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node('save_slam_map')
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    got = {}

    def on_tf(msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == args.frame:
                q = t.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                got['tf'] = (t.transform.translation.x, t.transform.translation.y, yaw)

    node.create_subscription(
        OccupancyGrid, args.topic, lambda m: got.__setitem__('map', m), latched)
    node.create_subscription(TFMessage, '/tf_static', on_tf, latched)
    deadline = time.monotonic() + args.wait
    while time.monotonic() < deadline and not ('map' in got and 'tf' in got):
        rclpy.spin_once(node, timeout_sec=0.2)
    if 'map' not in got:
        print(f'지도({args.topic})가 오지 않습니다. localization:=slam 으로 켰는지 확인하세요')
        return 1
    if 'tf' not in got:
        print(f'map -> {args.frame} 변환이 없습니다')
        return 1
    grid, transform = got['map'], got['tf']
    info = grid.info
    width, height, origin, pixels = to_map_frame(
        info.width, info.height, info.resolution,
        (info.origin.position.x, info.origin.position.y), grid.data, transform)
    path = write_map(args.out, width, height, info.resolution, origin, pixels,
                     note='save_slam_map 이 저장한 방 지도 (좌표 = 경기장 도면 좌표)')
    occupied = pixels.count(bytes([OCCUPIED]))
    print(f'저장: {path}  {width} x {height} 칸, {width * info.resolution:.1f} x '
          f'{height * info.resolution:.1f} m, 장애물 {occupied} 칸, '
          f'시작 자리 ({transform[0]:.2f}, {transform[1]:.2f}, {math.degrees(transform[2]):.0f}°)')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
