"""
카메라 영상에서 차선 중심을 찾는다 (ROS 와 무관한 계산만).

1. 어두운 픽셀(테이프)을 고른다.
2. 카메라 높이·기울기·화각으로 픽셀을 바닥 좌표로 바꾼다 (x = 앞, y = 왼쪽, 로봇 기준 m).
3. 앞으로 일정 간격의 띠마다 테이프 위치를 찾고, 가까운 띠부터 차선 중심을 따라간다.
   테이프가 하나만 보이면 차선 폭의 절반만큼 옮겨 중심을 정한다.

바닥 좌표로 계산하므로 차선 폭(m)과 카메라 값만 맞으면 실물과 시뮬레이션에 같은 코드를 쓴다.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class Camera:
    """카메라 위치와 화각. 로봇 기준(base_footprint), m·rad."""

    height: float
    x: float
    tilt: float     # 아래로 기울어진 각도
    hfov: float     # 수평 화각


@dataclass
class LaneResult:
    """차선 인식 결과."""

    found: bool
    target: tuple = (0.0, 0.0)  # 따라갈 차선 중심점 (x, y)
    near_offset: float = 0.0    # 가장 가까운 띠에서 차선 중심의 y (로봇이 오른쪽으로 치우치면 +)
    slices: int = 0             # 중심을 찾은 띠 개수


class GroundProjector:
    """픽셀 → 바닥 좌표. 영상 크기마다 표를 한 번 만들어 둔다."""

    def __init__(self, camera, width, height, max_range):
        f = (width / 2) / math.tan(camera.hfov / 2)
        u, v = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
        ray_y = -(u - width / 2) / f
        ray_z = -(v - height / 2) / f
        c, s = math.cos(camera.tilt), math.sin(camera.tilt)
        forward = c + s * ray_z
        down = s - c * ray_z
        with np.errstate(divide='ignore', invalid='ignore'):
            t = np.where(down > 1e-6, camera.height / down, np.nan)
        self.x = camera.x + t * forward
        self.y = t * ray_y
        self.valid = np.isfinite(self.x) & (self.x < max_range)

    def points(self, mask):
        """마스크(True = 테이프)의 바닥 좌표 (x 배열, y 배열)."""
        keep = mask & self.valid
        return self.x[keep], self.y[keep]


def tape_mask(gray, threshold):
    """어두운 픽셀 = 테이프."""
    return gray < threshold


def clusters(values, gap):
    """정렬한 값들을 gap 보다 크게 떨어진 곳에서 나눠 각 묶음의 평균을 돌려준다."""
    if len(values) == 0:
        return []
    values = np.sort(values)
    splits = np.where(np.diff(values) > gap)[0] + 1
    return [float(group.mean()) for group in np.split(values, splits)]


def _offset_from_tape(points, half, side):
    """
    테이프 한쪽만 보일 때의 차선 중심점.

    테이프와 수직 방향으로 차선 폭의 절반만큼 옮긴다. 곡선에서 옆(y)으로만 옮기면 중심이
    바깥쪽으로 밀린다. side: 왼쪽 테이프 = 1, 오른쪽 테이프 = -1.
    """
    x1, y1 = points[-1]
    heading = 0.0
    if len(points) > 1:
        x0, y0 = points[-2]
        heading = math.atan2(y1 - y0, x1 - x0)
    return x1 + side * half * math.sin(heading), y1 - side * half * math.cos(heading)


def find_lane(xs, ys, lane_width, near, far, step, start=0.0, tolerance=0.06):
    """
    가까운 띠부터 차선 중심을 따라가 LaneResult 를 돌려준다.

    start 는 가장 가까운 띠에서 기대하는 중심 y (보통 0 = 로봇 바로 앞, 차선 변경 때 옆 차선).
    """
    half = lane_width / 2
    center = start
    found, tapes_seen = [], {1: [], -1: []}
    for x in np.arange(near, far + 1e-9, step):
        x = float(x)
        band = np.abs(xs - x) < step / 2
        tapes = clusters(ys[band], gap=0.03)
        side_y = {}
        for side in (1, -1):
            expected = center + side * half
            near_tapes = [y for y in tapes if abs(y - expected) < tolerance]
            if near_tapes:
                side_y[side] = min(near_tapes, key=lambda y: abs(y - expected))
                tapes_seen[side].append((x, side_y[side]))
        if len(side_y) == 2:
            point = (x, (side_y[1] + side_y[-1]) / 2)
        elif side_y:
            side = next(iter(side_y))
            point = _offset_from_tape(tapes_seen[side], half, side)
        else:
            continue
        center = point[1]   # 다음 띠에서 테이프를 찾을 기준
        found.append(point)
    if len(found) < 2:
        return LaneResult(False, slices=len(found))
    return LaneResult(True, target=found[-1], near_offset=found[0][1], slices=len(found))


def pursuit_curvature(target):
    """로봇에서 target 점을 지나는 원호의 곡률 (pure pursuit). 왼쪽 회전이 +."""
    x, y = target
    return 2 * y / (x * x + y * y)
