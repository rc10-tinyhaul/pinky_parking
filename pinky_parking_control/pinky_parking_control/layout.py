"""
배치: 로봇이 다니는 곳의 좌표 (경기장 도면, 또는 지금 시뮬레이션용 공장 월드).

행동 단위(주차, 트랙 주행, 차선 변경, 주유)를 Nav2 목표 위치 목록으로 바꾸는 일을 한다.
ROS 를 import 하지 않으므로 ROS 없이 시험할 수 있다.
"""
import math

from pinky_parking_control.model import normalize_angle
from pinky_parking_control.model import parse_pose

ARENA, FACTORY = 'arena', 'factory'
FUEL_STEPS = ('wait', 'enter', 'leave')
WAYPOINTS_PER_LAP = 12   # 한 바퀴를 12 점으로 (1번 트랙 약 0.31 m 간격)
PATH_STEP = 0.03         # 연속 경로의 점 간격 [m]
LAP_END_GAP = 0.15       # 한 바퀴 경로를 출발점보다 이만큼 앞에서 끝낸다 (끝 = 시작이면 출발하자마자 도착으로 친다)
LANE_CHANGE_LENGTH = 0.6  # 차선을 옮기는 동안 나아가는 거리 [m]


class NotInLayout(Exception):
    """지금 배치에 없는 행동 (예: 공장 월드에서 트랙 주행)."""


class Stadium:
    """직선 + 반원 두 개로 된 트랙 중심선. 반시계 방향으로 잰 길이 s 로 위치를 낸다."""

    def __init__(self, radius, straight):
        self.r, self.h = float(radius), float(straight) / 2.0
        self.length = 4.0 * self.h + 2.0 * math.pi * self.r

    def point(self, s):
        """길이 s [m] 지점의 (x, y, yaw). s = 0 은 오른쪽 직선 아래 끝."""
        r, h = self.r, self.h
        s %= self.length
        if s < 2 * h:                                   # 오른쪽 직선, 위로
            return r, -h + s, math.pi / 2
        s -= 2 * h
        if s < math.pi * r:                             # 위 반원
            a = s / r
            return r * math.cos(a), h + r * math.sin(a), normalize_angle(a + math.pi / 2)
        s -= math.pi * r
        if s < 2 * h:                                   # 왼쪽 직선, 아래로
            return -r, h - s, -math.pi / 2
        s -= 2 * h
        a = math.pi + s / r                             # 아래 반원
        return r * math.cos(a), -h + r * math.sin(a), normalize_angle(a + math.pi / 2)

    def nearest(self, x, y, samples=720):
        """(x, y) 에 가장 가까운 중심선 위치의 s 와 거리."""
        best = min((math.hypot(px - x, py - y), s)
                   for s in (self.length * i / samples for i in range(samples))
                   for px, py, _ in [self.point(s)])
        return best[1], best[0]

    def pose(self, s):
        x, y, yaw = self.point(s)
        return {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)}


class ArenaLayout:
    """경기장 도면 (config/arena.yaml)."""

    kind = ARENA

    def __init__(self, data):
        self.data = data
        self.tracks = {int(t['id']): Stadium(t['radius'], data['straight'])
                       for t in data['tracks']}
        self.slots = {k: parse_pose(v) for k, v in data['slots'].items()}
        self.exits = {k: parse_pose(v) for k, v in data.get('exits', {}).items()}
        self.fuel = {k: parse_pose(data['fuel'][k]) for k in FUEL_STEPS}
        self.vehicles = data['vehicles']

    def track(self, track_id):
        try:
            return self.tracks[int(track_id)]
        except (KeyError, TypeError, ValueError):
            raise NotInLayout(f'없는 트랙입니다: {track_id} (있는 트랙: {list(self.tracks)})')

    def nearest_track(self, pose):
        return min(self.tracks, key=lambda t: self.tracks[t].nearest(pose['x'], pose['y'])[1])

    def drive(self, track_id, pose, laps=1):
        """지금 위치에서 가장 가까운 지점부터 반시계로 laps 바퀴 돌아 그 자리로 오는 경유점."""
        track = self.track(track_id)
        s0 = track.nearest(pose['x'], pose['y'])[0] if pose else 0.0
        n = WAYPOINTS_PER_LAP * max(1, int(laps))
        return [track.pose(s0 + track.length * i / WAYPOINTS_PER_LAP) for i in range(1, n + 1)]

    def lane_change(self, to_track, pose):
        """지금 트랙에서 같은 비율 위치의 앞쪽으로 옮겨 타는 경유점 두 개."""
        target = self.track(to_track)
        if not pose:
            raise ValueError('로봇 위치를 모릅니다 (AMCL 위치가 아직 없습니다)')
        current = self.tracks[self.nearest_track(pose)]
        fraction = current.nearest(pose['x'], pose['y'])[0] / current.length
        return [target.pose((fraction + ahead) * target.length) for ahead in (0.06, 0.12)]

    @staticmethod
    def _with_heading(points, last_yaw):
        """(x, y) 목록 → 진행 방향을 yaw 로 붙인 위치 목록."""
        out = []
        for i, (x, y) in enumerate(points):
            if i + 1 < len(points):
                nx, ny = points[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = last_yaw
            out.append({'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)})
        return out

    @staticmethod
    def _approach(pose, first):
        """로봇이 경로 시작점에서 떨어져 있으면 그 사이를 직선으로 잇는 점들."""
        gap = math.hypot(first[0] - pose['x'], first[1] - pose['y'])
        steps = int(gap / PATH_STEP)
        return [(pose['x'] + (first[0] - pose['x']) * i / steps,
                 pose['y'] + (first[1] - pose['y']) * i / steps) for i in range(steps)]

    def drive_path(self, track_id, pose, laps=1):
        """
        트랙 중심선을 따라 laps 바퀴 도는 촘촘한 경로 (Nav2 FollowPath 용).

        경유점마다 멈추지 않고 곡선을 이어서 달린다. 지금 위치에서 가장 가까운 지점부터 반시계.
        """
        track = self.track(track_id)
        s0 = track.nearest(pose['x'], pose['y'])[0] if pose else 0.0
        length = track.length * max(1, int(laps)) - LAP_END_GAP
        count = int(length / PATH_STEP)
        points = [track.point(s0 + length * i / count)[:2] for i in range(count + 1)]
        if pose:
            points = self._approach(pose, points[0]) + points
        return self._with_heading(points, track.point(s0 + length)[2])

    def lane_change_path(self, to_track, pose):
        """지금 트랙에서 옆 트랙으로 비스듬히 옮겨 타는 매끄러운 경로 (S 자)."""
        target = self.track(to_track)
        if not pose:
            raise ValueError('로봇 위치를 모릅니다 (AMCL 위치가 아직 없습니다)')
        current = self.tracks[self.nearest_track(pose)]
        fraction = current.nearest(pose['x'], pose['y'])[0] / current.length
        ahead = LANE_CHANGE_LENGTH / target.length
        count = int(LANE_CHANGE_LENGTH / PATH_STEP)
        points = []
        for i in range(count + 1):
            u = i / count
            w = u * u * (3 - 2 * u)                       # 0 → 1 로 부드럽게 (양 끝 기울기 0)
            ax, ay, _ = current.point((fraction + ahead * u) * current.length)
            bx, by, _ = target.point((fraction + ahead * u) * target.length)
            points.append((ax + (bx - ax) * w, ay + (by - ay) * w))
        points = self._approach(pose, points[0]) + points
        return self._with_heading(points, target.point((fraction + ahead) * target.length)[2])

    def fuel_poses(self, step):
        if step not in FUEL_STEPS:
            raise ValueError(f'주유 단계는 {", ".join(FUEL_STEPS)} 중 하나입니다: {step}')
        return [self.fuel[step]]

    def to_dict(self):
        out = dict(self.data)
        out['kind'] = ARENA
        out['routes'] = {f'track_{t}': [s.pose(s.length * i / 48) for i in range(48)]
                         for t, s in self.tracks.items()}
        return out


class FactoryLayout:
    """지금 시뮬레이션용 공식 공장 월드 (slots.yaml, route.yaml, 출차 임시 위치)."""

    kind = FACTORY

    def __init__(self, slots, routes, exits, vehicles):
        self.slots = {k: parse_pose(v) for k, v in (slots or {}).items()}
        self.routes = {k: [parse_pose(p) for p in v] for k, v in (routes or {}).items()}
        self.exits = {k: parse_pose(v) for k, v in (exits or {}).items()}
        self.vehicles = vehicles

    def drive(self, track_id, pose, laps=1):
        raise NotInLayout('트랙 주행은 경기장 배치에서만 됩니다 (지금은 공장 월드)')

    lane_change = drive_path = lane_change_path = drive

    def fuel_poses(self, step):
        raise NotInLayout('주유구역은 경기장 배치에서만 있습니다 (지금은 공장 월드)')

    def to_dict(self):
        return {'kind': FACTORY, 'name': '공장 월드 (시뮬레이션 임시)', 'slots': self.slots,
                'routes': self.routes, 'exits': self.exits, 'vehicles': self.vehicles}


def choose_layout(setting, world_name):
    """control.yaml 의 layout: arena | factory | auto. auto 는 월드 이름으로 고른다."""
    if setting in (ARENA, FACTORY):
        return setting
    return FACTORY if 'factory' in str(world_name) else ARENA
