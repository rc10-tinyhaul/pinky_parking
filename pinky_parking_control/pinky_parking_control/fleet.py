"""
로봇 전체를 행동 단위로 다룬다.

  기본 이동   goto, cancel, stop, 직접 운전(teleop)
  주차·출차   park, unpark
  주행        drive(트랙 1·2 한 바퀴), lane_change(차선 변경), lap(공장 월드 경로)
  주유        fuel(wait 대기선 → enter 주유구역 → leave 트랙 복귀)
  차종        vehicle(T00 · T10 · T15)

행동 하나는 Nav2 목표 하나(navigate_to_pose) 또는 경유점 목록(follow_waypoints)이 된다.
좌표는 배치(layout.py)가 정한다: 경기장 도면 또는 지금 시뮬레이션용 공장 월드.
"""
import threading
import time

from pinky_parking_control.layout import NotInLayout
from pinky_parking_control.model import NotYet
from pinky_parking_control.model import parse_pose
from pinky_parking_control.model import Task
from pinky_parking_control.model import TaskLog

# 기획에는 있지만 아직 없는 기능: 화면과 API 에 자리만 두고 "준비 중"으로 답한다
PENDING = {
    'gap': '앞뒤 간격 조절은 준비 중입니다 (#12)',
    'park_precise': '주차 칸 정밀 정렬은 준비 중입니다 (#7 마커 검출, #8 정밀 정렬)',
}
TELEOP_TIMEOUT_S = 0.5   # 직접 운전 명령이 이만큼 끊기면 멈춘다 (손을 떼면 선다)


class UnknownName(KeyError):
    """없는 로봇, 칸, 경로, 차종 이름."""

    def __str__(self):
        return self.args[0] if self.args else ''


class Fleet:
    def __init__(self, robots, layout, config, link_factory, callsigns=None):
        """
        로봇 목록과 배치로 함대를 만든다.

        robots: robots.yaml 의 로봇 목록. link_factory(name, domain_id, on_finish) 가
        로봇 연결을 만든다 (시험에서는 ROS 없이 가짜 연결을 넘긴다).
        """
        self.layout = layout
        self.timeouts = {'goto': 180.0, 'park': 180.0, 'unpark': 180.0, 'lap': 300.0,
                         'drive': 300.0, 'lane_change': 120.0, 'fuel': 180.0}
        self.timeouts.update({k: float(v) for k, v in (config.get('timeout_s') or {}).items()})
        teleop = config.get('teleop') or {}
        self.max_linear = float(teleop.get('max_linear', 0.15))
        self.max_angular = float(teleop.get('max_angular', 1.2))
        self.log = TaskLog()
        self.links = {r['name']: link_factory(r['name'], r['domain_id'], self.log.add)
                      for r in robots}
        names = list(self.links)
        self.callsigns = {name: sign for sign, name in (callsigns or {}).items() if name in names}
        defaults = config.get('vehicles') or {}
        self.vehicle = {n: defaults.get(n, 'T00') for n in names}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._watch = threading.Thread(target=self._watch_loop, daemon=True)
        self._watch.start()

    def _link(self, name):
        if name not in self.links:
            raise UnknownName(f'없는 로봇입니다: {name} (있는 로봇: {", ".join(self.links)})')
        return self.links[name]

    @staticmethod
    def _pick(table, key, kind):
        if key not in table:
            raise UnknownName(f'없는 {kind}입니다: {key} (있는 {kind}: {", ".join(table)})')
        return table[key]

    def _start(self, name, action, target, poses):
        link = self._link(name)
        return link.start(Task(robot=name, action=action, target=target, poses=poses,
                               timeout_s=self.timeouts[action]))

    def _pose(self, name):
        return self._link(name).status().get('pose')

    # ---- 기본 이동 -----------------------------------------------------
    def goto(self, name, pose):
        pose = parse_pose(pose)
        target = f'({pose["x"]:.2f}, {pose["y"]:.2f})'
        return self._start(name, 'goto', target, [pose])

    def cancel(self, name):
        return self._link(name).cancel()

    def stop(self, name):
        return self._link(name).stop()

    def stop_all(self):
        return {name: link.stop() for name, link in self.links.items()}

    def teleop(self, name, linear, angular):
        """직접 운전: 속도를 제한해 cmd_vel 로 낸다. 0.5 초 동안 새 명령이 없으면 멈춘다."""
        clamp = (lambda v, m: max(-m, min(m, float(v))))
        return self._link(name).teleop(clamp(linear, self.max_linear),
                                       clamp(angular, self.max_angular))

    # ---- 주차·출차 -----------------------------------------------------
    def park(self, name, slot):
        return self._start(name, 'park', slot, [self._pick(self.layout.slots, slot, '주차 칸')])

    def unpark(self, name, slot):
        return self._start(name, 'unpark', slot,
                           [self._pick(self.layout.exits, slot, '출차 위치')])

    # ---- 주행 ----------------------------------------------------------
    def drive(self, name, track, laps=1):
        poses = self.layout.drive(track, self._pose(name), laps)
        return self._start(name, 'drive', f'{track}번 트랙', poses)

    def lane_change(self, name, to_track):
        poses = self.layout.lane_change(to_track, self._pose(name))
        return self._start(name, 'lane_change', f'{to_track}번 트랙으로', poses)

    def lap(self, name, route):
        routes = getattr(self.layout, 'routes', {})
        return self._start(name, 'lap', route, self._pick(routes, route, '경로'))

    # ---- 주유 ----------------------------------------------------------
    def fuel(self, name, step):
        labels = {'wait': '대기선 30', 'enter': '주유구역 20', 'leave': '2번 트랙 복귀'}
        return self._start(name, 'fuel', labels.get(step, step), self.layout.fuel_poses(step))

    # ---- 차종 ----------------------------------------------------------
    def set_vehicle(self, name, vehicle_type):
        self._link(name)
        self._pick(self.layout.vehicles, vehicle_type, '차종')
        with self._lock:
            self.vehicle[name] = vehicle_type
        return self.vehicle_info(name)

    def vehicle_info(self, name):
        kind = self.vehicle[name]
        return {'type': kind, **self.layout.vehicles.get(kind, {})}

    # ---- 준비 중 -------------------------------------------------------
    @staticmethod
    def pending(feature):
        raise NotYet(PENDING[feature])

    # ---- 상태 조회 -----------------------------------------------------
    def robot(self, name):
        status = self._link(name).status()
        status['vehicle'] = self.vehicle_info(name)
        status['callsign'] = self.callsigns.get(name, name)
        return status

    def snapshot(self, log_limit=20):
        return {'time': time.time(), 'layout': self.layout.kind,
                'robots': [self.robot(name) for name in self.links],
                'log': self.log.recent(log_limit)}

    def layout_dict(self):
        out = self.layout.to_dict()
        out['pending'] = PENDING
        out['callsigns'] = self.callsigns
        out['teleop'] = {'linear': self.max_linear, 'angular': self.max_angular}
        return out

    # ---- 감시: 시간 초과, 직접 운전 끊김 --------------------------------
    def _watch_loop(self):
        expired = set()   # 이미 취소를 보낸 작업 (한 번만 보낸다)
        while not self._stop.wait(0.1):
            now = time.time()
            for link in self.links.values():
                task = link.task
                if task is not None and task.id not in expired and task.timed_out(now):
                    expired.add(task.id)
                    link.cancel(f'시간 초과 ({task.timeout_s:.0f}s)')
                link.teleop_watchdog(now, TELEOP_TIMEOUT_S)

    def close(self):
        self._stop.set()
        for link in self.links.values():
            link.close()


__all__ = ['Fleet', 'UnknownName', 'NotInLayout', 'PENDING']
