"""
로봇 전체를 행동 단위로 다룬다.

  기본 이동   goto, cancel, stop, 직접 운전(teleop)
  주차·출차   park, unpark
  주행        drive(트랙 1·2 한 바퀴), lane_change(차선 변경)
  주유        fuel(wait 대기선 → enter 주유구역 → leave 트랙 복귀)
  차종        vehicle(T00 · T10 · T15)

행동 하나는 Nav2 목표 하나(navigate_to_pose) 또는 경유점 목록(follow_waypoints)이 된다.
좌표는 배치(layout.py, 경기장 도면)가 정한다.
"""
import math
import threading
import time

from pinky_parking_control import health
from pinky_parking_control import reference
from pinky_parking_control import tuning
from pinky_parking_control.layout import NotInLayout
from pinky_parking_control.model import NotYet
from pinky_parking_control.model import parse_pose
from pinky_parking_control.model import save_trajectory
from pinky_parking_control.model import Task
from pinky_parking_control.model import TaskBusy
from pinky_parking_control.model import TaskLog

# 기획에는 있지만 이번 범위에서 뺀 기능: 화면과 API 에 자리만 두고 그렇게 답한다
PENDING = {
    'gap': '앞뒤 간격 조절은 이번 범위에서 뺐습니다 (#12)',
    'park_precise': '마커 기준 정렬 주차는 이번 범위에서 뺐습니다 (#8)',
}
TELEOP_TIMEOUT_S = 0.5   # 직접 운전 명령이 이만큼 끊기면 멈춘다 (손을 떼면 선다)
HOME_IN_SLOT = 0.08   # 시작 자리가 칸 가운데에서 이만큼 안이면 그 칸의 자리로 본다 [m] (화면의 '칸 안' 기준과 같음)
PRECISE_TOLERANCE = (0.03, 0.08)   # 주차·주유 진입의 도착 허용 오차 (거리 m, 방향 rad ≈ 5 도)
NORMAL_YAW_TOLERANCE = 0.25        # 그 밖의 이동에서 방향 허용 오차 (공식 값)
# 궤적 파일에 남길 주행 설정 중 Nav2 에서 읽는 것 (도착 허용 오차는 관제가 정해 보낸 값을 쓴다)
SETTINGS_SOURCES = {k: v for k, v in tuning.NAV2_SOURCES.items() if k != 'goal_tolerance'}


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
        self.timeouts = {'goto': 180.0, 'park': 180.0, 'unpark': 180.0,
                         'drive': 300.0, 'lane_change': 120.0, 'fuel': 180.0,
                         'reference': 600.0}
        self.timeouts.update({k: float(v) for k, v in (config.get('timeout_s') or {}).items()})
        teleop = config.get('teleop') or {}
        self.max_linear = float(teleop.get('max_linear', 0.15))
        self.max_angular = float(teleop.get('max_angular', 1.2))
        self.log = TaskLog()
        self.trajectory_dir = config.get('trajectory_dir') or ''
        self.tuning = {}                 # 관제 화면에서 바꾼 주행 설정 (바꾼 것만)
        # 로봇 위치가 확인됐는지: 켠 직후에는 '시작 자리에 있다'는 가정일 뿐이다.
        # 위치 맞추기를 하거나 마커로 한 번 보정되면 확인된 것으로 본다.
        self.pose_confirmed = {}
        self.stalls = {}                 # 로봇 이름 → 멈춤 감시 (명령을 받았는데 안 움직이는 시간)
        self.stalled = {}                # 로봇 이름 → 지금 멈춰 있는 시간 [s]
        # 로봇 이름 → 기준 경로 구간들 (가제보 등에서 기록한 주행 좌표. reference.py)
        self.references = {name: reference.load_reference(path)
                           for name, path in (config.get('references') or {}).items()}
        self.cameras = dict(config.get('cameras') or {})   # 로봇 이름 → 카메라 영상 주소
        self.links = {r['name']: link_factory(r['name'], r['domain_id'], self._finished)
                      for r in robots}
        # 시작 자리 (robots.yaml 의 스폰 위치 = 주차된 상태). '주차 자리로' 가 여기로 되돌린다
        self.home = {r['name']: {'x': float(r['x']), 'y': float(r['y']), 'yaw': float(r['yaw'])}
                     for r in robots if 'x' in r}
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

    def _start(self, name, action, target, poses, mode='', precise=False):
        link = self._link(name)
        tolerance = self._set_tolerance(link, precise)
        task = Task(robot=name, action=action, target=target, poses=poses,
                    timeout_s=self.timeouts[action], mode=mode,
                    settings=self._settings(link, tolerance), vehicle=self.vehicle.get(name, ''))
        return link.start(task)

    def _set_tolerance(self, link, precise):
        """
        도착 허용 오차를 행동에 맞게 바꾼다. 정한 값 (거리 m, 방향 rad) 을 돌려준다.

        주차·주유 진입은 칸(폭 190 mm)에 맞춰 좁게, 그 밖의 이동은 화면에서 정한 값(느슨하게)으로.
        좁은 값으로 트랙을 돌면 도착 판정이 까다로워 끝에서 머뭇거린다.
        """
        normal = self.tuning.get('goal_tolerance', tuning.SETTINGS['goal_tolerance'][5])
        xy, yaw = PRECISE_TOLERANCE if precise else (normal, NORMAL_YAW_TOLERANCE)
        if hasattr(link, 'set_nav2_parameters'):
            link.set_nav2_parameters({'controller_server': {
                'general_goal_checker.xy_goal_tolerance': xy,
                'general_goal_checker.yaw_goal_tolerance': yaw}})
        return xy, yaw

    def _settings(self, link, tolerance):
        """
        작업을 시작할 때의 주행 설정 (궤적 파일에 함께 남겨 튜닝 로그로 쓴다).

        속도·회전·주시 거리는 Nav2 에서 지금 값을 읽는다 (launch 인자로 정한 값은 Nav2 만 안다).
        화면에서 바꾼 값이 있으면 그 값이 이긴다 (보낸 직후에는 Nav2 가 옛 값을 줄 수 있다).
        못 읽은 값은 넣지 않는다. 도착 허용 오차는 방금 보낸 값이다.
        """
        values = {}
        if hasattr(link, 'get_nav2_parameters'):
            values.update(link.get_nav2_parameters(SETTINGS_SOURCES, timeout_s=0.3))
        values.update({k: v for k, v in self.tuning.items() if k in SETTINGS_SOURCES})
        values['goal_tolerance'], values['yaw_tolerance'] = tolerance
        return values

    def _finished(self, task):
        """작업이 끝나면 기록에 넣고, 궤적 폴더가 정해져 있으면 궤적 파일을 남긴다."""
        if task.mode == 'path':      # 계획 경로에서 얼마나 벗어났는지 (로봇이 계산한 위치 기준)
            task.deviation = reference.deviation(task.poses, task.trail)
        # 실물인지: 배터리 값은 실물 로봇만 낸다 (pose_confirmed 와 같은 기준). 시작 때는 아직
        # 배터리 값이 안 왔을 수 있어 끝날 때 본다.
        link = self.links.get(task.robot)
        if link is not None:
            task.real = link.status().get('battery_v') is not None
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
        center = self._pick(self.layout.slots, slot, '주차 칸')
        poses = [center]
        if hasattr(self.layout, 'park_poses'):       # 경기장: 칸 뒤에 먼저 서서 똑바로 들어간다
            poses = self.layout.park_poses(center, self._pose(name))
        return self._start(name, 'park', slot, poses, precise=True)

    def unpark(self, name, slot):
        # 출차한 자리가 곧 트랙 출발점이라 정밀하게 선다 (느슨하면 차선 밖에서 출발한다)
        exit_poses = self._pick(self.layout.exits, slot, '출차 위치')
        if not isinstance(exit_poses, list):
            exit_poses = [exit_poses]
        return self._start(name, 'unpark', slot, exit_poses, precise=True)

    # ---- 주행 ----------------------------------------------------------
    def drive(self, name, track, laps=1):
        poses = self.layout.drive_path(track, self._pose(name), laps)
        return self._start(name, 'drive', f'{track}번 트랙', poses, mode='path')

    def lane_change(self, name, to_track):
        poses = self.layout.lane_change_path(to_track, self._pose(name))
        return self._start(name, 'lane_change', f'{to_track}번 트랙으로', poses, mode='path')

    # ---- 주유 ----------------------------------------------------------
    def fuel(self, name, step):
        labels = {'wait': '대기선 30', 'enter': '주유구역 20', 'leave': '2번 트랙 복귀'}
        return self._start(name, 'fuel', labels.get(step, step), self.layout.fuel_poses(step),
                           precise=(step == 'enter'))

    # ---- 기준 경로 따라가기 --------------------------------------------
    def reference_view(self, name):
        """이 로봇의 기준 경로 구간들 (좌표는 빼고 요약만)."""
        self._link(name)
        return [{'index': i, 'name': seg['name'], 'steps': seg['steps'], 'length': seg['length'],
                 'duration': seg['duration'], 'start': seg['poses'][0], 'end': seg['poses'][-1]}
                for i, seg in enumerate(self.references.get(name, []))]

    def follow_reference(self, name, segment):
        """기준 경로의 구간 하나를 그대로 따라간다. 마지막 구간은 주차로 끝나므로 정밀하게 선다."""
        segments = self.references.get(name)
        if not segments:
            raise UnknownName(f'{name} 의 기준 경로가 없습니다 (서버의 --reference 인자)')
        try:
            index = int(segment)
            seg = segments[index]
        except (TypeError, ValueError, IndexError):
            raise UnknownName(f'없는 구간입니다: {segment} (0 ~ {len(segments) - 1})')
        if index < 0:
            raise UnknownName(f'없는 구간입니다: {segment}')
        poses = reference.with_approach(seg['poses'], self._pose(name))
        return self._start(name, 'reference', f'기준 경로 {seg["name"]}', poses, mode='path',
                           precise=(index == len(segments) - 1))

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
        """로봇을 손으로 옮겨 놓은 뒤 '지금 여기 있다'고 알려 준다. slot 이면 그 칸의 자리(_slot_pose)."""
        link = self._link(name)
        if link.task is not None and not link.task.done:
            raise TaskBusy(f'{name} 은(는) 작업 중입니다. 먼저 멈추세요')
        target = self._slot_pose(name, slot) if slot else parse_pose(pose)
        link.set_pose(target)
        self.pose_confirmed[name] = True
        return {'robot': name, 'pose': target}

    def _slot_pose(self, name, slot):
        """
        로봇이 주차 칸에 서 있을 때의 자리.

        그 로봇의 시작 자리(robots.yaml)가 이 칸 안이면 시작 자리를 쓴다. B 는 10번 칸에서 아래를 보고
        칸 가운데보다 5 cm 아래에 서므로, 칸 가운데·칸 방향을 쓰면 방향이 반대가 된다.
        그 밖에는 칸 가운데와 칸 방향이다.
        """
        center = self._pick(self.layout.slots, slot, '주차 칸')
        home = self.home.get(name)
        if home and math.hypot(home['x'] - center['x'], home['y'] - center['y']) <= HOME_IN_SLOT:
            return dict(home)
        return center

    def reset_home(self, teleport=None):
        """
        모든 로봇을 시작 자리(주차 상태)로 되돌린다: 작업을 멈추고 위치를 그 자리로 맞춘다.

        teleport(name, pose) 가 True 를 돌려주면 (시뮬레이션) 로봇 모델도 그 자리로 옮긴 것이다.
        실물은 옮길 수 없으므로 사람이 로봇을 칸에 놓은 뒤 누른다 (위치만 맞춘다).
        """
        if not self.home:
            raise NotInLayout('robots.yaml 에 시작 자리(x, y, yaw)가 없습니다')
        out = []
        for name, link in self.links.items():
            link.cancel('주차 자리로 되돌림')
        for name, pose in self.home.items():
            if name not in self.links:
                continue
            moved = bool(teleport(name, pose)) if teleport else False
            self.links[name].set_pose(pose)
            self.pose_confirmed[name] = True
            self.stalled.pop(name, None)
            out.append({'robot': name, 'pose': pose, 'moved': moved})
        return {'robots': out, 'sim': any(r['moved'] for r in out)}

    def register_markers(self, name, slot=''):
        """
        지금 카메라에 보이는 마커의 지도 위치를 등록한다.

        slot 을 주면 로봇이 그 주차 칸의 자리(_slot_pose)에 있다고 보고(가장 정확), 비우면 로봇이 지금 믿는
        위치를 쓴다 (트랙 위의 마커: 칸에서 위치를 맞춘 직후 가까운 마커부터 차례로 등록한다).
        """
        link = self._link(name)
        status = link.status().get('marker')
        if not status:
            raise NotYet('마커 위치 보정이 꺼져 있습니다 (real.launch.py 의 cameras 인자)')
        if not status.get('visible'):
            raise ValueError('지금 카메라에 보이는 마커가 없습니다')
        if slot:
            target = self._slot_pose(name, slot)
            self.pose_confirmed[name] = True
        else:
            target = self._pose(name)
            if not target:
                raise ValueError('로봇 위치를 모릅니다')
        link.register_markers(target)
        return {'robot': name, 'pose': target, 'markers': status['visible']}

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

    # ---- 범위에서 뺀 기능 -------------------------------------------------------
    @staticmethod
    def pending(feature):
        raise NotYet(PENDING[feature])

    # ---- 상태 조회 -----------------------------------------------------
    def robot(self, name):
        status = self._link(name).status()
        status['vehicle'] = self.vehicle_info(name)
        status['callsign'] = self.callsigns.get(name, name)
        status['lane'] = self.lane_status(status.get('pose'))
        marker = status.get('marker') or {}
        # 마커로 한 번 보정됐거나, 등록된 마커가 보이고 그 위치와 맞는지 비교되고 있으면 확인된 것
        if (marker.get('last') or {}).get('id') is not None or marker.get('offset') is not None:
            self.pose_confirmed[name] = True
        # 시뮬레이션(배터리 값이 없다)은 스폰 위치가 곧 진짜 위치라 처음부터 확인된 것으로 본다
        status['pose_confirmed'] = self.pose_confirmed.get(name, status.get('battery_v') is None)
        status['stalled_s'] = round(self.stalled.get(name, 0.0), 1)
        status['alerts'] = health.alerts(status, self.stalled.get(name, 0.0), time.time())
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
        # 기준 경로를 화면에 그릴 점들 (구간마다 [[x, y], ...])
        out['references'] = {name: [[[p['x'], p['y']] for p in seg['poses'][::2]] for seg in segs]
                             for name, segs in self.references.items()}
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
            for name, link in self.links.items():      # 명령을 받았는데 움직이지 않는지
                task = link.task
                running = task is not None and not task.done
                watch = self.stalls.setdefault(name, health.StallWatch())
                self.stalled[name] = watch.update(
                    task.id if running else None, getattr(link, 'pose', None), now)

    def close(self):
        self._stop.set()
        for link in self.links.values():
            link.close()


__all__ = ['Fleet', 'UnknownName', 'NotInLayout', 'PENDING']
