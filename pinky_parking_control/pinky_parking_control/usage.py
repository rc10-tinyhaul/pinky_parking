"""
로봇 사용량 누적 (바퀴 기록 odom 과 배터리로 계산). 관제 서버를 켠 뒤부터 센다.

핑키프로는 바퀴 토크(모터 전류)를 내보내지 않는다 (/joint_states 의 effort 가 비어 있다).
그래서 토크 대신 바퀴마다 '가감속 누적'(속도 변화량의 합)을 센다. 모터 토크는 대부분 차를
빠르게·느리게 할 때 쓰이므로, 이 값이 크면 그만큼 바퀴에 힘을 많이 쓴 것이다 (추정치).
"""
import math

WHEEL_RADIUS = 0.028        # 핑키프로 바퀴 반지름 [m] (pinky_description)
WHEEL_SEPARATION = 0.0961   # 두 바퀴 사이 [m]
MOVING_SPEED = 0.01         # 이보다 빠르면(직진 m/s 또는 바퀴 m/s) 달리는 중으로 본다
MAX_GAP_S = 0.5             # odom 이 이보다 오래 끊기면 그 사이는 세지 않는다


class Usage:
    def __init__(self):
        self.distance = 0.0               # 달린 거리 [m]
        self.turn = 0.0                   # 돈 각도 합 [rad]
        self.drive_s = 0.0                # 움직인 시간 [s]
        self.wheel = [0.0, 0.0]           # 바퀴마다 굴러간 거리 [m] (왼쪽, 오른쪽)
        self.effort = [0.0, 0.0]          # 바퀴마다 가감속 누적 [m/s] (토크 추정)
        self.battery_start = None         # 처음 받은 배터리 전압 [V]
        self.battery_low = None           # 지금까지 가장 낮은 전압 [V]
        self._last = None                 # (시각, 왼쪽 바퀴 속도, 오른쪽 바퀴 속도)

    def on_odom(self, stamp, linear, angular):
        half = angular * WHEEL_SEPARATION / 2
        wheels = (linear - half, linear + half)
        last, self._last = self._last, (stamp, *wheels)
        if last is None:
            return
        dt = stamp - last[0]
        if not 0.0 < dt <= MAX_GAP_S:
            return
        self.distance += abs(linear) * dt
        self.turn += abs(angular) * dt
        if max(abs(w) for w in wheels) > MOVING_SPEED:
            self.drive_s += dt
        for i in (0, 1):
            self.wheel[i] += abs(wheels[i]) * dt
            self.effort[i] += abs(wheels[i] - last[i + 1])

    def on_battery(self, volts):
        if self.battery_start is None:
            self.battery_start = volts
        self.battery_low = volts if self.battery_low is None else min(self.battery_low, volts)

    def to_dict(self, battery_now=None):
        used = None
        if self.battery_start is not None and battery_now is not None:
            used = round(self.battery_start - battery_now, 2)
        return {
            'distance_m': round(self.distance, 2),
            'turn_deg': round(math.degrees(self.turn)),
            'drive_s': round(self.drive_s),
            'wheel_rev': [round(w / (2 * math.pi * WHEEL_RADIUS), 1) for w in self.wheel],
            'effort': [round(e, 1) for e in self.effort],
            'battery_start_v': (round(self.battery_start, 2)
                                if self.battery_start is not None else None),
            'battery_used_v': used,
        }
