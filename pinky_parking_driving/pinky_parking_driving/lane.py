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
    offset: float = 0.0         # 로봇에서 차선 중심선까지 수직 거리 (중심선이 왼쪽이면 +)
    heading: float = 0.0        # 로봇 정면에 대한 차선 방향 [rad] (차선이 왼쪽으로 향하면 +)


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
    # 찾은 중심점의 처음과 끝을 잇는 선 = 차선 중심선. 로봇이 비스듬히 서 있으면
    # near_offset 은 실제 거리보다 작게 나오므로, 수직 거리와 방향을 따로 낸다.
    (x0, y0), (x1, y1) = found[0], found[-1]
    heading = math.atan2(y1 - y0, x1 - x0)
    offset = y0 * math.cos(heading) - x0 * math.sin(heading)
    return LaneResult(True, target=found[-1], near_offset=found[0][1], slices=len(found),
                      offset=offset, heading=heading)


def pursuit_curvature(target):
    """로봇에서 target 점을 지나는 원호의 곡률 (pure pursuit). 왼쪽 회전이 +."""
    x, y = target
    return 2 * y / (x * x + y * y)


class LaneChanger:
    """
    차선 변경 상태 (ROS 와 무관).

    요청은 목표 차선 번호다. 반시계 방향 주행이라 번호가 큰 차선이 오른쪽(-y)에 있다.
    직선 구간은 길이가 짧아(0.6 m) 변경에 약 0.47 m 가 들므로, 곡선이 끝나고 직선이
    시작되는 곳에서만 시작한다. 변경 중에는 find_lane 의 start(차선을 찾는 기준)를
    목표 차선 중심으로 옮기고, 조향은 평소 차선 추종 그대로 둔다.

    상태 문자열: idle:<차선>, waiting:<목표>, changing:<목표>, done:<목표>, failed:<목표>:<이유>
    """

    def __init__(self, lane_width, lane=None, lanes=2, straight_curvature=0.8,
                 straight_frames=5, start_after=0.0, max_start_distance=0.10,
                 done_offset=0.015, done_near=0.04, done_frames=5, max_distance=0.7,
                 handover_offset=0.05, wait_distance=10.0):
        self.width = lane_width
        self.lane = lane
        self.lanes = lanes
        self.straight_curvature = straight_curvature
        self.straight_frames = straight_frames
        self.start_after = start_after
        self.max_start_distance = max_start_distance
        self.done_offset = done_offset
        self.done_near = done_near
        self.done_frames = done_frames
        self.handover_offset = handover_offset
        self.max_distance = max_distance
        self.wait_distance = wait_distance
        self.state = 'idle'
        self.target = None
        self._start = 0.0
        self._in_curve = False
        self._straight_count = 0
        self._gate = None          # 이번 직선이 시작된 곳의 주행 거리
        self._requested_at = None
        self._changed_at = None
        self._done_count = 0

    def status(self, reason=None):
        if self.state == 'idle':
            return f'idle:{self.lane or 0}'
        if reason:
            return f'failed:{self.target}:{reason}'
        return f'{self.state}:{self.target}'

    def request(self, target, distance=0.0):
        """목표 차선 요청. 바로 낼 상태 문자열을 돌려준다 (None = 알릴 것 없음)."""
        if self.state in ('waiting', 'changing') and target == self.target:
            return None                                   # 같은 요청이 다시 옴
        if self.state == 'changing':
            return f'failed:{target}:busy'
        if self.lane is None:
            return f'failed:{target}:no_lane'
        if not 1 <= target <= self.lanes:
            return f'failed:{target}:invalid'
        if target == self.lane:
            self.state, self.target = 'idle', None
            return f'done:{target}'
        if abs(target - self.lane) != 1:
            return f'failed:{target}:not_adjacent'
        self.state, self.target = 'waiting', target
        self._requested_at = distance
        return self.status()

    def start_y(self):
        """이번 영상에서 find_lane 에 줄 start."""
        return self._start

    def update(self, result, curvature, distance):
        """
        차선을 찾은 영상마다 부른다. 상태가 바뀌면 상태 문자열을, 아니면 None 을 돌려준다.

        distance 는 누적 주행 거리 [m].
        """
        if not result.found:
            return None
        self._track_straight(curvature, distance)
        if self.state == 'waiting':
            return self._update_waiting(result, distance)
        if self.state == 'changing':
            return self._update_changing(result, distance)
        return None

    def _track_straight(self, curvature, distance):
        if abs(curvature) >= self.straight_curvature:
            self._in_curve, self._straight_count = True, 0
            return
        self._straight_count += 1
        if self._in_curve and self._straight_count >= self.straight_frames:
            self._in_curve, self._gate = False, distance

    def _update_waiting(self, result, distance):
        if self._gate is not None:
            since = distance - self._gate
            if since > self.max_start_distance:
                self._gate = None                         # 늦었음: 다음 직선을 기다림
            elif since >= self.start_after:
                direction = -1 if self.target > self.lane else 1
                self._start = result.near_offset + direction * self.width
                self.state, self._changed_at, self._done_count = 'changing', distance, 0
                self._gate = None
                return self.status()
        if distance - self._requested_at > self.wait_distance:
            event = self.status('no_straight')
            self.state, self.target = 'idle', None
            return event
        return None

    def _update_changing(self, result, distance):
        self._start = result.near_offset                  # 목표 차선 중심을 계속 따라감
        # 끝: 중심선까지 수직 거리가 작고, 바로 앞의 중심이 평소 추종(start = 0)이 찾을 수 있는 안
        if abs(result.offset) < self.done_offset and abs(result.near_offset) < self.done_near:
            self._done_count += 1
        else:
            self._done_count = 0
        if self._done_count >= self.done_frames:
            return self._finish()
        if distance - self._changed_at > self.max_distance:
            # 곡선에 들어가면 중심선 추정(offset)이 곡률만큼 틀어져 위 조건이 맞지 않는다.
            # 목표 차선 안에 충분히 들어왔으면 평소 추종에 넘기고, 아니면 멈춘다.
            if abs(result.near_offset) < self.handover_offset:
                return self._finish()
            return self.fail('too_long')
        return None

    def _finish(self):
        event = f'done:{self.target}'
        self.lane, self.state, self.target, self._start = self.target, 'idle', None, 0.0
        return event

    def fail(self, reason):
        """변경을 그만둔다 (차선을 잃었을 때 등). 상태 문자열을 돌려준다."""
        event = self.status(reason) if self.state in ('waiting', 'changing') else None
        if self.state == 'changing':
            self.lane = None                              # 차선 사이에서 멈춤: 차선을 모름
        self.state, self.target, self._start = 'idle', None, 0.0
        return event
