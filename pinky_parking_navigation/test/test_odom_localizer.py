import math

from pinky_parking_navigation.odom_localizer import compose
from pinky_parking_navigation.odom_localizer import inverse
from pinky_parking_navigation.odom_localizer import map_to_odom
import pytest


def close(a, b):
    return all(math.isclose(p, q, abs_tol=1e-9) for p, q in zip(a, b))


def test_start_pose_maps_odom_origin():
    start = (0.0, 0.15, math.pi / 2)                 # 11번 주차 칸, 위를 봄
    offset = map_to_odom(start, (0.0, 0.0, 0.0))     # 켤 때 odom = 0
    assert close(offset, start)
    # 로봇이 앞으로 0.3 m 가면 (odom x + 0.3) 지도에서는 y 가 0.3 늘어난다
    assert close(compose(offset, (0.3, 0.0, 0.0)), (0.0, 0.45, math.pi / 2))


def test_nonzero_odom_at_start_and_reset():
    odom = (1.0, -2.0, 0.7)                          # 재시작 등으로 odom 이 0 이 아닐 때
    start = (-0.595, 0.1, -math.pi / 2)
    assert close(compose(map_to_odom(start, odom), odom), start)
    assert close(compose(inverse(odom), odom), (0.0, 0.0, 0.0))


@pytest.mark.parametrize('yaw', [-3.1, -1.0, 0.0, 2.0, 3.1])
def test_compose_keeps_angle_in_range(yaw):
    assert -math.pi <= compose((0, 0, yaw), (0, 0, yaw))[2] <= math.pi
