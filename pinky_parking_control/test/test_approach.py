"""주차 칸 마지막 직선 진입(approach) 검사."""
import math

from pinky_parking_control import approach
from pinky_parking_control.robot_link import compose_delta
import pytest

SLOT_10 = {'x': 0.0, 'y': -0.15, 'yaw': math.pi / 2}    # 아래에서 위로 들어간다


@pytest.mark.parametrize('start', [
    {'x': 0.0, 'y': -0.43, 'yaw': math.pi / 2},
    {'x': 0.03, 'y': -0.43, 'yaw': math.pi / 2 + 0.08},      # 옆 3 cm, 방향 5°
    {'x': -0.03, 'y': -0.45, 'yaw': math.pi / 2 - 0.30},
    {'x': 0.0, 'y': -0.40, 'yaw': math.pi / 2 + 0.6},       # 34° 틀어짐: 먼저 돈다
])
def test_reaches_slot_center(start):
    pose, state = approach.simulate(start, SLOT_10)
    remaining, side, heading = approach.errors(pose, SLOT_10)
    assert state == 'done'
    assert abs(remaining) < 0.005 and abs(side) < 0.005
    assert abs(heading) <= approach.ALIGN_TOLERANCE


def test_never_backs_up_and_stops_after_passing():
    v, w, state = approach.command({'x': 0.0, 'y': -0.14, 'yaw': math.pi / 2 + 0.2}, SLOT_10)
    assert state == 'align' and v == 0.0 and w < 0           # 지났으면 방향만 맞춘다
    for y in (-0.43, -0.3, -0.2):
        v, _, _ = approach.command({'x': 0.0, 'y': y, 'yaw': math.pi / 2}, SLOT_10)
        assert approach.MIN_LINEAR <= v <= approach.MAX_LINEAR


def test_compose_delta_adds_odom_motion_in_map():
    """바퀴 기록(odom)에서 앞으로 0.1 m 간 것은 지도에서 로봇이 보는 방향(+y)으로 0.1 m 간 것이다."""
    pose = {'x': 0.0, 'y': -0.43, 'yaw': math.pi / 2}
    then, now = (1.0, 2.0, 0.3), (1.0 + 0.1 * math.cos(0.3), 2.0 + 0.1 * math.sin(0.3), 0.3)
    moved = compose_delta(pose, then, now)
    assert moved['x'] == pytest.approx(0.0, abs=1e-9)
    assert moved['y'] == pytest.approx(-0.33, abs=1e-9)
    assert moved['yaw'] == pytest.approx(math.pi / 2)
