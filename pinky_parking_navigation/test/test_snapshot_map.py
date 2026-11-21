"""제자리 라이다 스냅샷으로 지도 만들기(snapshot_map) 검사."""
import math

from pinky_parking_navigation.snapshot_map import build_map
from pinky_parking_navigation.snapshot_map import FREE
from pinky_parking_navigation.snapshot_map import OCCUPIED
from pinky_parking_navigation.snapshot_map import scan_hits
from pinky_parking_navigation.snapshot_map import UNKNOWN
import pytest


def cell(width, height, origin, pixels, x, y, res):
    col = int(math.floor((x - origin[0]) / res))
    row = height - 1 - int(math.floor((y - origin[1]) / res))
    return pixels[row * width + col]


def test_scan_hits_uses_robot_pose_and_rear_facing_lidar():
    # 로봇이 (0, 0.15) 에서 위(+y)를 본다. 라이다는 180° 돌아 있어 각도 π 의 빛이 로봇 앞으로 나간다
    origin, hits = scan_hits([1.0], math.pi, 0.0, (0.0, 0.15, math.pi / 2), max_range=6.0)
    assert origin == pytest.approx((0.0, 0.133), abs=1e-3)
    assert hits[0] == pytest.approx((0.0, 1.133), abs=1e-3)


def test_wall_kept_and_passerby_dropped():
    res, pose = 0.05, (0.0, 0.0, 0.0)
    wall = [2.0, 2.0, 2.0]                    # 세 방향 모두 2 m 에 벽
    scans = [(wall, math.pi - 0.02, 0.02)] * 9
    scans.append(([0.8, 2.0, 2.0], math.pi - 0.02, 0.02))      # 한 번만 가까이에 사람이 찍힘
    width, height, origin, pixels, walls = build_map(scans, pose, res, keep=0.4)
    assert cell(width, height, origin, pixels, 1.98, 0.0, res) == OCCUPIED     # 벽
    assert cell(width, height, origin, pixels, 0.78, -0.02, res) == FREE       # 사람은 지워지고 빈 곳
    assert cell(width, height, origin, pixels, 1.0, 0.0, res) == FREE          # 로봇과 벽 사이
    assert cell(width, height, origin, pixels, 2.2, 0.0, res) == UNKNOWN       # 벽 뒤는 모름
    assert walls >= 1


def test_empty_scan_raises():
    with pytest.raises(ValueError):
        build_map([([float('inf')] * 10, 0.0, 0.1)], (0.0, 0.0, 0.0))
