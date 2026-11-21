"""
Nav2 지도(map_server 의 YAML + PGM)를 대시보드가 그릴 수 있는 형태로 읽는다.

cells 는 한 칸에 한 글자: 'o' 장애물, 'f' 빈 곳, 'u' 모름. 맨 윗줄(y 가 가장 큰 줄)부터 차례로.
"""
import math
from pathlib import Path

import yaml


def read_pgm(data):
    """P5 PGM 바이트 → (width, height, maxval, pixels)."""
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    if tokens[0] != b'P5':
        raise ValueError('P5 PGM 만 읽습니다')
    width, height, maxval = (int(t) for t in tokens[1:])
    pixels = data[pos + 1:pos + 1 + width * height]
    if len(pixels) != width * height:
        raise ValueError('PGM 크기가 맞지 않습니다')
    return width, height, maxval, pixels


def classify(pixels, maxval, negate, occupied_thresh, free_thresh):
    """map_server 의 trinary 규칙으로 칸을 나눈다."""
    out = []
    for value in pixels:
        p = value / maxval if negate else (maxval - value) / maxval
        out.append('o' if p > occupied_thresh else 'f' if p < free_thresh else 'u')
    return ''.join(out)


def load_map(yaml_path):
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    width, height, maxval, pixels = read_pgm((yaml_path.parent / meta['image']).read_bytes())
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    return {
        'width': width, 'height': height, 'resolution': float(meta['resolution']),
        'origin': {'x': float(origin[0]), 'y': float(origin[1])},
        'cells': classify(pixels, maxval, int(meta.get('negate', 0)),
                          float(meta.get('occupied_thresh', 0.65)),
                          float(meta.get('free_thresh', 0.196))),
    }


LIDAR_X, LIDAR_YAW = -0.017, 3.14159265   # base_footprint → rplidar_link (공식 URDF: 180° 돌아 있다)
SCAN_POINTS = 240            # 화면에 보낼 라이다 점 개수 상한


def encode_grid(msg):
    """ROS OccupancyGrid 를 화면용으로 바꾼다. cells 는 한 칸에 한 글자 (o 장애물, f 빈 곳, u 모름), 맨 윗줄부터."""
    w, h = msg.info.width, msg.info.height
    table = {True: 'o', False: 'f'}
    rows = []
    for r in range(h - 1, -1, -1):
        row = msg.data[r * w:(r + 1) * w]
        rows.append(''.join('u' if v < 0 else table[v > 50] for v in row))
    return {'width': w, 'height': h, 'resolution': msg.info.resolution,
            'origin': {'x': msg.info.origin.position.x, 'y': msg.info.origin.position.y},
            'cells': ''.join(rows)}


def scan_points(ranges, angle_min, angle_increment, range_max, pose, limit=SCAN_POINTS):
    """라이다 거리값 → 지도 좌표의 점 [[x, y], ...] (로봇 위치 pose 기준, 점 개수는 limit 이하)."""
    step = max(1, len(ranges) // limit)
    cos_r, sin_r = math.cos(pose['yaw']), math.sin(pose['yaw'])
    points = []
    for i in range(0, len(ranges), step):
        d = ranges[i]
        if not (0.05 < d < range_max) or d != d:
            continue
        a = angle_min + i * angle_increment + LIDAR_YAW
        bx, by = LIDAR_X + d * math.cos(a), d * math.sin(a)
        points.append([round(pose['x'] + cos_r * bx - sin_r * by, 3),
                       round(pose['y'] + sin_r * bx + cos_r * by, 3)])
    return points
