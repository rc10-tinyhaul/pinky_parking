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

from pinky_parking_control import tuning
from pinky_parking_control.layout import NotInLayout
from pinky_parking_control.model import NotYet
from pinky_parking_control.model import parse_pose
from pinky_parking_control.model import save_trajectory
from pinky_parking_control.model import Task
from pinky_parking_control.model import TaskBusy
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
        self.trajectory_dir = config.get('trajectory_dir') or ''
        self.tuning = {}                 # 관제 화면에서 바꾼 주행 설정 (바꾼 것만)
        self.cameras = dict(config.get('cameras') or {})   # 로봇 이름 → 카메라 영상 주소
        self.links = {r['name']: link_factory(r['name'], r['domain_id'], self._finished)
                      for r in robots}
        names = list(self.links)
        for name, url in self.cameras.items():
            if name in self.links:
                self.links[name].camera_url = url
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

    def _start(self, name, action, target, poses, mode=''):
        link = self._link(name)
        return link.start(Task(robot=name, action=action, target=target, poses=poses,
                               timeout_s=self.timeouts[action], mode=mode))

    def _finished(self, task):
        """작업이 끝나면 기록에 넣고, 궤적 폴더가 정해져 있으면 궤적 파일을 남긴다."""
        self.log.add(task)
        if self.trajectory_dir:
            try:
                save_trajectory(task, self.trajectory_dir)
            except OSError as error:      # 기록 실패가 로봇 제어를 막지 않게
                print(f'궤적을 저장하지 못했습니다: {error}')

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
        poses = self.layout.drive_path(track, self._pose(name), laps)
        return self._start(name, 'drive', f'{track}번 트랙', poses, mode='path')

    def lane_change(self, name, to_track):
        poses = self.layout.lane_change_path(to_track, self._pose(name))
        return self._start(name, 'lane_change', f'{to_track}번 트랙으로', poses, mode='path')

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

    # ---- 위치 다시 맞추기 ----------------------------------------------
    def set_pose(self, name, pose=None, slot=None):
        """로봇을 손으로 옮겨 놓은 뒤 '지금 여기 있다'고 알려 준다. slot 이면 그 주차 칸 가운데."""
        link = self._link(name)
        if link.task is not None and not link.task.done:
            raise TaskBusy(f'{name} 은(는) 작업 중입니다. 먼저 멈추세요')
        target = self._pick(self.layout.slots, slot, '주차 칸') if slot else parse_pose(pose)
        link.set_pose(target)
        return {'robot': name, 'pose': target}

    # ---- 주행 설정 (속도, 감도) ----------------------------------------
    def tuning_view(self):
        """지금 설정. 화면에서 아직 안 바꾼 값은 Nav2 에서 읽어 온다 (못 읽으면 기본값)."""
        values = {'teleop_linear': self.max_linear, 'teleop_angular': self.max_angular}
        for link in self.links.values():
            if hasattr(link, 'get_nav2_parameters'):
                values.update(link.get_nav2_parameters(tuning.NAV2_SOURCES))
                break
        values.update(self.tuning)
        return tuning.describe(values)

    def set_tuning(self, changes):
        """설정을 바꿔 모든 로봇의 Nav2 에 바로 적용한다. 속도와 회전은 함께 다시 보낸다."""
        cleaned = tuning.clean(changes)
        self.tuning.update(cleaned)
        if 'teleop_linear' in cleaned:
            self.max_linear = cleaned['teleop_linear']
        if 'teleop_angular' in cleaned:
            self.max_angular = cleaned['teleop_angular']
        current = {item['key']: item['value'] for item in tuning.describe(self.tuning)}
        if 'speed' in cleaned or 'turn' in cleaned:
            cleaned = {**cleaned, 'speed': current['speed'], 'turn': current['turn']}
        by_node = tuning.nav2_parameters({k: v for k, v in cleaned.items()
                                          if k in tuning.NAV2_KEYS})
        applied = {name: link.set_nav2_parameters(by_node)
                   for name, link in self.links.items()
                   if by_node and hasattr(link, 'set_nav2_parameters')}
        return {'values': tuning.describe({**current, **self.tuning}), 'applied': applied}

    def lane_status(self, pose):
        """경기장에서 가장 가까운 트랙과 그 중심선에서 벗어난 거리 [m]. 경기장이 아니면 None."""
        if not pose or not hasattr(self.layout, 'nearest_track'):
            return None
        track_id = self.layout.nearest_track(pose)
        offset = self.layout.tracks[track_id].nearest(pose['x'], pose['y'], samples=360)[1]
        return {'track': track_id, 'offset': round(offset, 3)}

    # ---- 준비 중 -------------------------------------------------------
    @staticmethod
    def pending(feature):
        raise NotYet(PENDING[feature])

    # ---- 상태 조회 -----------------------------------------------------
    def robot(self, name):
        status = self._link(name).status()
        status['vehicle'] = self.vehicle_info(name)
        status['callsign'] = self.callsigns.get(name, name)
        status['lane'] = self.lane_status(status.get('pose'))
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
