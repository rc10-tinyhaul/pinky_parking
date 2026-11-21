"""
시연 시나리오 실행기 (config/scenario.yaml).

한 단계 안에서 로봇끼리는 동시에, 같은 로봇의 행동은 적힌 순서대로 한다.
단계 하나씩(다음 단계) 또는 끝까지 이어서(전체 실행) 돌릴 수 있다.
"""
import threading
import time

from pinky_parking_control.model import SUCCEEDED

READY, RUNNING, DONE, FAILED, SKIPPED, STOPPED = (
    'ready', 'running', 'done', 'failed', 'skipped', 'stopped')
ACTION_WORDS = {'park': '주차', 'unpark': '출차', 'drive': '트랙 주행', 'lane_change': '차선 변경',
                'fuel': '주유', 'goto': '이동', 'vehicle': '차종', 'confirm': '사람 확인'}
FUEL_WORDS = {'wait': '대기선 30', 'enter': '주유구역 20', 'leave': '트랙 복귀'}


def describe(action):
    """행동 하나를 사람이 읽는 문장으로."""
    do, who = action.get('do'), action.get('robot', '?')
    if do == 'park' or do == 'unpark':
        what = action.get('slot', '')
    elif do == 'drive':
        what = f'{action.get("track")}번 트랙 1바퀴'
    elif do == 'lane_change':
        what = f'{action.get("to")}번 트랙으로'
    elif do == 'fuel':
        what = FUEL_WORDS.get(action.get('step'), action.get('step', ''))
    elif do == 'vehicle':
        what = action.get('type', '')
    elif do == 'confirm':
        what = action.get('text', '')
    elif do == 'goto':
        what = f'({action.get("x")}, {action.get("y")})'
    else:
        what = ''
    return f'{who} {ACTION_WORDS.get(do, do)} {what}'.strip()


class ScenarioRunner:
    def __init__(self, fleet, data, poll_s=0.2):
        self.fleet = fleet
        self.callsigns = dict(data.get('callsigns') or {})
        self.steps = list(data.get('steps') or [])
        self.poll_s = poll_s
        self.states = [READY] * len(self.steps)
        self.notes = [''] * len(self.steps)
        self.current = None
        self.mode = None
        self.waiting = None          # 사람 확인을 기다리는 문장
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._confirm = threading.Event()
        self._thread = None

    # ---- 명령 ----------------------------------------------------------
    def start(self, index=None, run_all=False):
        """단계를 하나 또는 끝까지 실행한다. index 가 없으면 다음 준비된 단계부터."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError('시나리오가 이미 실행 중입니다')
            if index is None:
                again = (READY, STOPPED, FAILED)
                index = next((i for i, s in enumerate(self.states) if s in again), None)
                if index is None:
                    raise ValueError('남은 단계가 없습니다. 처음부터 하려면 초기화하세요')
            index = int(index)
            if not 0 <= index < len(self.steps):
                raise ValueError(f'없는 단계입니다: {index + 1}')
            self._stop.clear()
            self._confirm.clear()
            self.mode = 'all' if run_all else 'step'
            self._thread = threading.Thread(target=self._run, args=(index, run_all), daemon=True)
            self._thread.start()
        return self.status()

    def stop(self):
        """멈추고 로봇의 작업을 취소한다."""
        self._stop.set()
        self._confirm.set()
        for name in self.callsigns.values():
            if name in self.fleet.links:
                self.fleet.cancel(name)
        return self.status()

    def confirm(self):
        if self.waiting is None:
            raise ValueError('확인을 기다리는 단계가 없습니다')
        self._confirm.set()
        return self.status()

    def reset(self):
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError('실행 중에는 초기화할 수 없습니다. 먼저 멈추세요')
        self.states = [READY] * len(self.steps)
        self.notes = [''] * len(self.steps)
        self.current = None
        return self.status()

    def wait(self, timeout=None):
        """시험용: 실행이 끝날 때까지 기다린다."""
        if self._thread is not None:
            self._thread.join(timeout)

    # ---- 상태 ----------------------------------------------------------
    def status(self):
        return {
            'running': self._thread is not None and self._thread.is_alive(),
            'mode': self.mode, 'current': self.current, 'waiting': self.waiting,
            'callsigns': self.callsigns,
            'steps': [{'index': i, 'title': s.get('title', f'{i + 1}단계'),
                       'detail': s.get('detail', ''), 'pending': s.get('pending'),
                       'state': self.states[i], 'note': self.notes[i],
                       'actions': [describe(a) for a in s.get('actions') or []]}
                      for i, s in enumerate(self.steps)],
        }

    # ---- 실행 ----------------------------------------------------------
    def _run(self, index, run_all):
        for i in range(index, len(self.steps)):
            if self._stop.is_set():
                break
            self.current = i
            ok = self._execute(i)
            if not ok or not run_all:
                break
        self.current = None
        self.waiting = None

    def _execute(self, i):
        step = self.steps[i]
        if step.get('pending'):
            self.states[i], self.notes[i] = SKIPPED, f'건너뜀: {step["pending"]} 준비 중'
            return True
        self.states[i], self.notes[i] = RUNNING, ''
        queues, absent = {}, []
        for action in step.get('actions') or []:
            sign = action.get('robot')
            if self.callsigns.get(sign, sign) not in self.fleet.links:
                if sign not in absent:       # 연결하지 않은 로봇(예: 한 대만 시험)의 행동은 건너뛴다
                    absent.append(sign)
                continue
            queues.setdefault(sign, []).append(action)
        missing = f'{", ".join(str(s) for s in absent)} 없음: 그 행동은 건너뜀' if absent else ''
        if absent and not queues:
            self.states[i], self.notes[i] = SKIPPED, missing
            return True
        results = {}
        workers = [threading.Thread(target=self._run_queue, args=(sign, q, results), daemon=True)
                   for sign, q in queues.items()]
        for w in workers:
            w.start()
        for w in workers:
            w.join()
        failures = [msg for msg in results.values() if msg]
        if self._stop.is_set():
            self.states[i], self.notes[i] = STOPPED, '멈춤'
            return False
        if failures:
            self.states[i], self.notes[i] = FAILED, failures[0]
            return False
        self.states[i], self.notes[i] = DONE, missing
        return True

    def _run_queue(self, sign, actions, results):
        """한 로봇의 행동을 차례로 한다. 실패하면 이유를 results[sign] 에 남긴다."""
        name = self.callsigns.get(sign, sign)
        for action in actions:
            if self._stop.is_set():
                return
            try:
                message = self._do(name, action)
            except Exception as error:  # noqa: B902  어떤 실패든 단계 실패로 기록한다
                message = f'{describe(action)}: {error}'
            if message:
                results[sign] = message
                return

    def _do(self, name, action):
        """행동 하나. 성공하면 빈 문자열, 실패하면 이유."""
        do, f = action.get('do'), self.fleet
        if do == 'vehicle':
            f.set_vehicle(name, action['type'])
            return ''
        if do == 'confirm':
            self.waiting = action.get('text', '확인하세요')
            self._confirm.wait()
            self.waiting = None
            self._confirm.clear()
            return '' if not self._stop.is_set() else '멈춤'
        starters = {
            'park': lambda: f.park(name, action['slot']),
            'unpark': lambda: f.unpark(name, action['slot']),
            'drive': lambda: f.drive(name, action['track'], action.get('laps', 1)),
            'lane_change': lambda: f.lane_change(name, action['to']),
            'fuel': lambda: f.fuel(name, action['step']),
            'goto': lambda: f.goto(name, action),
        }
        if do not in starters:
            return f'모르는 행동입니다: {do}'
        task = starters[do]()
        while not task.done and not self._stop.is_set():
            time.sleep(self.poll_s)
        if task.state != SUCCEEDED:
            return f'{describe(action)}: {task.detail or task.state}'
        return ''
