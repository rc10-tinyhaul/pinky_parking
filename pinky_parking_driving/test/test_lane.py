"""차선 인식(lane.py) 검사. 영상 대신 바닥 좌표의 테이프 점을 만들어 넣는다."""
import math

import numpy as np
from pinky_parking_driving.lane import Camera
from pinky_parking_driving.lane import find_lane
from pinky_parking_driving.lane import GroundProjector
from pinky_parking_driving.lane import pursuit_curvature

WIDTH = 0.19
TAPE = 0.02
ARGS = {'lane_width': WIDTH, 'near': 0.17, 'far': 0.26, 'step': 0.02}


def tape_points(curve, offsets, x_max=0.5):
    """curve(s) → (x, y, heading) 를 따라 테이프 점을 촘촘히 만든다 (offsets: 중심선에서 왼쪽 거리)."""
    xs, ys = [], []
    for s in np.arange(0.0, 0.8, 0.002):
        x, y, heading = curve(s)
        for offset in offsets:
            for w in np.linspace(-TAPE / 2, TAPE / 2, 5):
                d = offset + w
                px, py = x - d * math.sin(heading), y + d * math.cos(heading)
                if 0.1 < px < x_max:
                    xs.append(px)
                    ys.append(py)
    return np.array(xs), np.array(ys)


def straight(shift):
    return lambda s: (s, shift, 0.0)


def left_turn(radius, shift=0.0):
    """로봇 앞에서 반경 radius 로 왼쪽으로 도는 중심선 (로봇은 중심선에서 shift 만큼 오른쪽)."""
    def curve(s):
        a = s / radius
        return radius * math.sin(a), radius * (1 - math.cos(a)) + shift, a
    return curve


def test_straight_centered():
    xs, ys = tape_points(straight(0.0), (WIDTH / 2, -WIDTH / 2))
    result = find_lane(xs, ys, **ARGS)
    assert result.found
    assert abs(result.near_offset) < 0.003
    assert abs(pursuit_curvature(result.target)) < 0.1


def test_straight_robot_right_of_center():
    xs, ys = tape_points(straight(0.03), (WIDTH / 2, -WIDTH / 2))
    result = find_lane(xs, ys, **ARGS)
    assert result.found
    assert abs(result.near_offset - 0.03) < 0.003
    assert pursuit_curvature(result.target) > 0   # 왼쪽으로 돌아 중심으로


def test_curve_only_outer_tape_visible():
    # 안쪽(왼쪽) 테이프가 화면 밖일 때: 바깥 테이프에서 수직으로 옮긴 점이 중심선 위에 있어야 함
    radius = 0.405
    xs, ys = tape_points(left_turn(radius), (-WIDTH / 2,))
    result = find_lane(xs, ys, **ARGS)
    assert result.found
    x, y = result.target
    assert abs(math.hypot(x, y - radius) - radius) < 0.005
    assert abs(1 / pursuit_curvature(result.target) - radius) < 0.02


def test_no_tape():
    result = find_lane(np.array([]), np.array([]), **ARGS)
    assert not result.found


def test_projector_matches_sim_camera():
    # Pinky Pro 시뮬레이션 카메라(1280x720): 영상 맨 아래 가운데는 로봇 앞 약 0.151 m
    camera = Camera(height=0.063, x=0.033, tilt=math.radians(8), hfov=1.1519)
    projector = GroundProjector(camera, 1280, 720, max_range=0.5)
    assert abs(projector.x[-1, 640] - 0.151) < 0.002
    assert abs(projector.y[-1, 640]) < 0.001
    assert not projector.valid[0, 640]   # 하늘
