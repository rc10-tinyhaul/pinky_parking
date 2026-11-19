"""
로봇 전체를 행동 단위로 다룬다: 기본 이동, 주차·출차, 주행, 상태 조회.

행동 하나는 Nav2 목표 하나(goto, park, unpark) 또는 경유점 목록(lap)이 된다.
좌표는 모두 각 패키지의 config YAML 에서 읽는다 (slots.yaml, route.yaml, control.yaml).
"""
import threading
import time

from pinky_parking_control.model import parse_pose
from pinky_parking_control.model import Task
from pinky_parking_control.model import TaskLog


class UnknownName(KeyError):
    """없는 로봇, 칸, 경로 이름."""

    def __str__(self):
        return self.args[0] if self.args else ''


class Fleet:
    def __init__(self, robots, slots, routes, config, link_factory):
        """
        robots: robots.yaml 의 로봇 목록. link_factory(name, domain_id, on_finish) 가 연결을 만든다.

        시험에서는 ROS 없이 가짜 연결을 넘긴다.
        """
        self.slots = {k: parse_pose(v) for k, v in (slots or {}).items()}
        self.routes = {k: [parse_pose(p) for p in v] for k, v in (routes or {}).items()}
        self.exits = {k: parse_pose(v) for k, v in (config.get('unpark_exits') or {}).items()}
        self.timeouts = {'goto': 180.0, 'park': 180.0, 'unpark': 180.0, 'lap': 300.0}
        self.timeouts.update({k: float(v) for k, v in (config.get('timeout_s') or {}).items()})
        self.log = TaskLog()
        self.links = {r['name']: link_factory(r['name'], r['domain_id'], self.log.add)
                      for r in robots}
        self._stop = threading.Event()
        self._watch = threading.Thread(target=self._watch_timeouts, daemon=True)
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

    # ---- 행동 단위 -----------------------------------------------------
    def goto(self, name, pose):
        pose = parse_pose(pose)
        target = f'({pose["x"]:.2f}, {pose["y"]:.2f})'
        return self._start(name, 'goto', target, [pose])

    def park(self, name, slot):
        return self._start(name, 'park', slot, [self._pick(self.slots, slot, '주차 칸')])

    def unpark(self, name, slot):
        return self._start(name, 'unpark', slot, [self._pick(self.exits, slot, '출차 위치')])

    def lap(self, name, route):
        return self._start(name, 'lap', route, self._pick(self.routes, route, '경로'))

    def cancel(self, name):
        return self._link(name).cancel()

    def stop(self, name):
        return self._link(name).stop()

    def stop_all(self):
        return {name: link.stop() for name, link in self.links.items()}

    # ---- 상태 조회 -----------------------------------------------------
    def robot(self, name):
        return self._link(name).status()

    def snapshot(self, log_limit=20):
        return {'time': time.time(),
                'robots': [link.status() for link in self.links.values()],
                'log': self.log.recent(log_limit)}

    def layout(self):
        return {'slots': self.slots, 'routes': self.routes, 'exits': self.exits}

    # ---- 시간 초과 감시 ------------------------------------------------
    def _watch_timeouts(self):
        expired = set()   # 이미 취소를 보낸 작업 (한 번만 보낸다)
        while not self._stop.wait(0.5):
            for link in self.links.values():
                task = link.task
                if task is not None and task.id not in expired and task.timed_out():
                    expired.add(task.id)
                    link.cancel(f'시간 초과 ({task.timeout_s:.0f}s)')

    def close(self):
        self._stop.set()
        for link in self.links.values():
            link.close()
