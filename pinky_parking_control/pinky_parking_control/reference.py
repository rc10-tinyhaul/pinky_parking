"""
기준 경로: 다른 곳(가제보 시뮬레이션 등)에서 기록한 주행 좌표를 실제 로봇이 따라갈 경로로 바꾼다.

CSV 한 줄 = 한 시각의 위치. 열: t_s, step_name, x_mm, y_mm, yaw_rad (그 밖의 열은 무시).
좌표는 경기장 도면 좌표(원점 = 트랙 중심, +x 오른쪽, +y 위)여야 한다. 우리 map 좌표와 같다.

- 멈춘 곳(같은 자리에 STOP_S 초 이상)에서 구간을 나눈다. 구간 하나가 Nav2 follow_path 하나가 된다.
- 구간 안에서는 PATH_STEP 간격으로 다시 뽑는다 (기록은 수 mm 간격이라 너무 촘촘하다).
- 실제 주행이 기준 경로에서 얼마나 벗어났는지 잰다 (deviation).

ROS 를 import 하지 않는다.
"""
import csv
import math
from pathlib import Path

PATH_STEP = 0.03      # 다시 뽑는 간격 [m]
STOP_S = 0.8          # 이 시간 이상 같은 자리에 있으면 멈춘 것으로 본다
STOP_RADIUS = 0.005   # '같은 자리'의 반경 [m]
MIN_LENGTH = 0.05     # 이보다 짧은 구간은 버린다


def read_samples(path):
    """CSV → [(t, x, y, yaw, step_name)] (단위 s, m, rad)."""
    samples = []
    with Path(path).open(encoding='utf-8', newline='') as handle:
        for row in csv.DictReader(handle):
            samples.append((float(row['t_s']), float(row['x_mm']) / 1000.0,
                            float(row['y_mm']) / 1000.0, float(row['yaw_rad']),
                            row.get('step_name', '')))
    if not samples:
        raise ValueError(f'기준 경로가 비어 있습니다: {path}')
    return samples


def split_at_stops(samples):
    """멈춘 곳을 경계로 움직인 구간들을 돌려준다 (각 구간은 samples 의 조각)."""
    moving, current = [], []
    i, n = 0, len(samples)
    while i < n:
        j = i
        while (j + 1 < n and math.hypot(samples[j + 1][1] - samples[i][1],
                                        samples[j + 1][2] - samples[i][2]) < STOP_RADIUS):
            j += 1
        if samples[j][0] - samples[i][0] >= STOP_S:        # i ~ j 는 멈춘 구간
            if current:
                current.append(samples[i])
                moving.append(current)
            current = [samples[j]]
            i = j + 1
        else:
            current.append(samples[i])
            i += 1
    if len(current) > 1:
        moving.append(current)
    return moving


def resample(points, step=PATH_STEP):
    """[(x, y, yaw)] 를 step 간격으로 다시 뽑는다. 끝점은 꼭 넣는다."""
    out = [points[0]]
    for p in points[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= step:
            out.append(p)
    if out[-1] != points[-1]:
        out.append(points[-1])
    return out


def load_reference(path):
    """
    기준 경로 CSV → 구간 목록.

    구간: {'name', 'steps' (지나는 단계 이름들), 'poses' [{x, y, yaw}], 'length' [m], 'duration' [s]}.
    """
    segments = []
    for chunk in split_at_stops(read_samples(path)):
        points = resample([(s[1], s[2], s[3]) for s in chunk])
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
        if length < MIN_LENGTH:
            continue
        steps = []
        for s in chunk:
            if s[4] and s[4] not in steps:
                steps.append(s[4])
        segments.append({
            'name': f'{len(segments) + 1}구간', 'steps': steps,
            'poses': [{'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)}
                      for x, y, yaw in points],
            'length': round(length, 2), 'duration': round(chunk[-1][0] - chunk[0][0], 1)})
    if not segments:
        raise ValueError(f'움직인 구간이 없습니다: {path}')
    return segments


def with_approach(poses, pose, step=PATH_STEP):
    """로봇이 구간 시작점에서 떨어져 있으면 그 사이를 직선으로 이어 붙인다."""
    if not pose:
        return poses
    first = poses[0]
    gap = math.hypot(first['x'] - pose['x'], first['y'] - pose['y'])
    count = int(gap / step)
    if count < 2:
        return poses
    yaw = math.atan2(first['y'] - pose['y'], first['x'] - pose['x'])
    lead = [{'x': round(pose['x'] + (first['x'] - pose['x']) * i / count, 4),
             'y': round(pose['y'] + (first['y'] - pose['y']) * i / count, 4),
             'yaw': round(yaw, 4)} for i in range(count)]
    return lead + poses


def deviation(planned, actual):
    """
    실제 궤적이 계획 경로에서 벗어난 거리 [m]: {'mean', 'max', 'end'}. 못 재면 None.

    planned: [{x, y}], actual: [[t, x, y, ...]]. 실제 점마다 가장 가까운 계획 선분까지의 거리.
    end 는 마지막 실제 위치와 계획의 끝점 사이 거리.
    """
    if len(planned) < 2 or not actual:
        return None
    segments = [((a['x'], a['y']), (b['x'], b['y'])) for a, b in zip(planned, planned[1:])]

    def to_path(x, y):
        best = float('inf')
        for (ax, ay), (bx, by) in segments:
            dx, dy = bx - ax, by - ay
            norm = dx * dx + dy * dy
            t = 0.0 if norm == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / norm))
            best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
        return best

    stride = max(1, len(actual) // 300)               # 긴 궤적은 300 점쯤만 잰다
    errors = [to_path(row[1], row[2]) for row in actual[::stride]]
    last, goal = actual[-1], planned[-1]
    return {'mean': round(sum(errors) / len(errors), 3), 'max': round(max(errors), 3),
            'end': round(math.hypot(last[1] - goal['x'], last[2] - goal['y']), 3)}
