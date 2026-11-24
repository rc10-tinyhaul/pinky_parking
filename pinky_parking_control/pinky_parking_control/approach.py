"""
주차 칸 마지막 직선 진입: 칸 입구에서 칸 가운데까지 관제가 직접 속도 명령을 낸다.

Nav2 는 도착 허용 오차(3 cm, 약 5°) 안에 들면 바로 멈추고, 곧게 들어가는 동안 방향을 바로잡지
않는다. 시뮬레이션에서 10번 칸에 10번 주차하면 평균 19 ~ 34 mm, 방향 최대 12° 어긋났다(#8).
그래서 입구까지만 Nav2 로 가고, 마지막 직선은 칸 가운데 줄을 따라가며 옆 어긋남과 방향을 계속
고치고, 정확한 거리에서 멈춘 뒤 방향을 마지막으로 맞춘다. ROS 를 import 하지 않는다 (시험용).

    command(pose, target) → (선속도, 각속도, 상태)    상태: 'turn' | 'drive' | 'align' | 'done'
"""
import math

MAX_LINEAR = 0.08        # 직선 진입 최고 속도 [m/s]
MIN_LINEAR = 0.02        # 멈추기 직전 최저 속도 [m/s] (너무 느리면 실물 바퀴가 안 돈다)
SLOW_GAIN = 0.8          # 남은 거리 [m] → 속도
MAX_ANGULAR = 0.6        # [rad/s]
LATERAL_GAIN = 8.0       # 옆 어긋남 1 cm → 목표 방향 약 4.6° 틀기 (Stanley 방식)
HEADING_GAIN = 2.5       # 방향 오차 [rad] → 각속도
STOP_DISTANCE = 0.003    # 남은 거리가 이보다 작으면 멈춘다 [m]
TURN_FIRST = 0.35        # 방향이 이보다 틀어져 있으면(약 20°) 먼저 제자리에서 돈다 [rad]
ALIGN_TOLERANCE = 0.012  # 멈춘 뒤 방향을 이 안(약 0.7°)으로 맞춘다 [rad]
MIN_TURN = 0.15          # 제자리 회전 최저 각속도 [rad/s]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def errors(pose, target):
    """목표 자리 기준: (남은 거리 [m], 옆 어긋남 [m, + = 왼쪽], 방향 오차 [rad, + = 왼쪽으로 틀어짐])."""
    dx, dy = pose['x'] - target['x'], pose['y'] - target['y']
    c, s = math.cos(target['yaw']), math.sin(target['yaw'])
    along = dx * c + dy * s
    side = -dx * s + dy * c
    return -along, side, wrap(pose['yaw'] - target['yaw'])


def turn_toward(error):
    """제자리 회전 각속도: 오차를 줄이는 쪽으로, 너무 느리지 않게."""
    w = clamp(-HEADING_GAIN * error, -MAX_ANGULAR, MAX_ANGULAR)
    return math.copysign(max(abs(w), MIN_TURN), w)


def command(pose, target, arrived=False):
    """
    지금 자리에서 낼 속도와 상태.

    arrived: 이미 거리상 도착해 방향만 맞추는 중인지. 거리상 도착하면 다시 앞으로 가지 않는다
    (회전 중 바퀴가 조금 밀려도 왔다 갔다 하지 않게).
    """
    remaining, side, heading = errors(pose, target)
    if arrived or remaining <= STOP_DISTANCE:
        if abs(heading) <= ALIGN_TOLERANCE:
            return 0.0, 0.0, 'done'
        return 0.0, turn_toward(heading), 'align'
    desired = -math.atan(LATERAL_GAIN * side)          # 가운데 줄로 돌아가는 방향 (target 기준)
    error = wrap(heading - desired)
    if abs(heading) > TURN_FIRST:
        return 0.0, turn_toward(heading), 'turn'
    v = clamp(SLOW_GAIN * remaining, MIN_LINEAR, MAX_LINEAR)
    w = clamp(-HEADING_GAIN * error, -MAX_ANGULAR, MAX_ANGULAR)
    return v, w, 'drive'


def simulate(start, target, dt=0.05, steps=2000, noise=None):
    """시험용: 이상적인 차동 구동 로봇으로 command 를 따라간 끝 자리와 상태."""
    pose, arrived = dict(start), False
    for _ in range(steps):
        v, w, state = command(pose, target, arrived)
        if state == 'done':
            return pose, state
        arrived = arrived or state == 'align'
        pose = {'x': pose['x'] + v * math.cos(pose['yaw']) * dt,
                'y': pose['y'] + v * math.sin(pose['yaw']) * dt,
                'yaw': wrap(pose['yaw'] + w * dt)}
        if noise:
            pose = noise(pose)
    return pose, 'timeout'
