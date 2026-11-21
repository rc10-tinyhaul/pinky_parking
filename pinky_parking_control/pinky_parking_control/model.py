"""
관제 API 의 순수 로직: 작업 상태, 좌표 변환, 작업 기록.

ROS 를 import 하지 않으므로 ROS 없이 시험할 수 있다 (test/test_model.py).
"""
from collections import deque
from dataclasses import dataclass
from dataclasses import field
import itertools
import math
import threading
import time

# 작업 상태
RUNNING = 'running'
SUCCEEDED = 'succeeded'
FAILED = 'failed'
CANCELED = 'canceled'
FINISHED = (SUCCEEDED, FAILED, CANCELED)

# 행동 단위 (Nav2 목표가 되는 것)
ACTIONS = ('goto', 'park', 'unpark', 'lap', 'drive', 'lane_change', 'fuel')


class TaskBusy(Exception):
    """로봇이 이미 다른 작업을 하고 있다."""


class NotReady(Exception):
    """로봇의 Nav2 가 아직 켜지지 않았다."""


class NotYet(Exception):
    """기획에는 있지만 아직 만들지 않은 기능 (담당 이슈가 끝나면 채운다)."""


def yaw_to_quaternion(yaw):
    """평면 회전각 [rad] → (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def quaternion_to_yaw(x, y, z, w):
    """(x, y, z, w) → 평면 회전각 [rad], -pi..pi."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_pose(data):
    """{'x', 'y', 'yaw'} 를 검사해 float 로 바꾼다. yaw 는 없으면 0."""
    try:
        pose = {'x': float(data['x']), 'y': float(data['y']),
                'yaw': float(data.get('yaw', 0.0))}
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f'위치는 x, y, yaw 숫자여야 합니다: {data}') from error
    if not all(math.isfinite(v) for v in pose.values()):
        raise ValueError(f'위치에 유한하지 않은 값이 있습니다: {data}')
    pose['yaw'] = normalize_angle(pose['yaw'])
    return pose


_task_ids = itertools.count(1)


@dataclass
class Task:
    """로봇 한 대의 작업 하나. 목표 위치(poses)를 차례로 지난다."""

    robot: str
    action: str
    target: str
    poses: list
    timeout_s: float
    id: int = field(default_factory=lambda: next(_task_ids))  # noqa: A003  API 의 작업 번호
    state: str = RUNNING
    started: float = field(default_factory=time.time)
    finished: float = None
    detail: str = ''
    distance_remaining: float = None
    waypoint_index: int = 0
    # '' = 위치 하나면 navigate_to_pose, 여럿이면 follow_waypoints. 'path' = follow_path (이어진 경로)
    mode: str = ''
    trail: list = field(default_factory=list)   # 달리며 기록한 실제 위치 [t, x, y, yaw, v, w]

    def __post_init__(self):
        if self.action not in ACTIONS:
            raise ValueError(f'모르는 행동입니다: {self.action}')
        if not self.poses:
            raise ValueError('목표 위치가 하나 이상 있어야 합니다')

    @property
    def done(self):
        return self.state in FINISHED

    def finish(self, state, detail='', now=None):
        """작업을 끝낸다. 이미 끝난 작업이면 False (먼저 온 결과가 이긴다)."""
        if state not in FINISHED:
            raise ValueError(f'끝 상태가 아닙니다: {state}')
        if self.done:
            return False
        self.state, self.detail = state, detail
        self.finished = time.time() if now is None else now
        return True

    def timed_out(self, now=None):
        now = time.time() if now is None else now
        return not self.done and now - self.started > self.timeout_s

    def elapsed(self, now=None):
        end = self.finished if self.done else (time.time() if now is None else now)
        return max(0.0, end - self.started)

    def to_dict(self, now=None):
        return {
            'id': self.id, 'robot': self.robot, 'action': self.action, 'target': self.target,
            'state': self.state, 'detail': self.detail, 'mode': self.mode,
            'started': self.started, 'finished': self.finished,
            'elapsed_s': round(self.elapsed(now), 1),
            'goal': self.poses[min(self.waypoint_index, len(self.poses) - 1)],
            'poses': self.poses,
            'waypoint_index': self.waypoint_index, 'waypoint_total': len(self.poses),
            'distance_remaining': (None if self.distance_remaining is None
                                   else round(self.distance_remaining, 3)),
        }


class TaskLog:
    """끝난 작업의 기록 (최근 것 먼저). 여러 스레드에서 쓴다."""

    def __init__(self, size=200):
        self._items = deque(maxlen=size)
        self._lock = threading.Lock()

    def add(self, task):
        with self._lock:
            self._items.appendleft(task.to_dict())

    def recent(self, limit=50):
        with self._lock:
            return list(self._items)[:max(0, int(limit))]


def save_trajectory(task, directory):
    """
    끝난 작업의 계획 경로와 실제 궤적을 JSON 파일 하나로 남긴다 (보정·분석용).

    planned: 보낸 목표 위치들 {x, y, yaw}.  actual: [작업 시작부터 초, x, y, yaw, 직진 m/s, 회전 rad/s].
    actual 은 로봇이 스스로 계산한 위치다(AMCL 또는 바퀴 기록). 실제 바닥 위치와의 차이는 따로 잰다.
    """
    import json
    from pathlib import Path
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y%m%d-%H%M%S', time.localtime(task.started))
    path = directory / f'{stamp}_{task.robot}_{task.id:03d}_{task.action}.json'
    data = {
        'robot': task.robot, 'action': task.action, 'target': task.target, 'mode': task.mode,
        'state': task.state, 'detail': task.detail, 'started': task.started,
        'elapsed_s': round(task.elapsed(), 2), 'planned': task.poses,
        'actual_columns': ['t', 'x', 'y', 'yaw', 'v', 'w'], 'actual': task.trail,
    }
    path.write_text(json.dumps(data, ensure_ascii=False), encoding='utf-8')
    return path
