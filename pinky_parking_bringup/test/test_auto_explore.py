"""자동 탐색의 판단 로직 시험 (ROS 없이)."""
import math

from pinky_parking_bringup import auto_explore as ex

N = 720
INC = 2 * math.pi / N


def scan(obstacles=(), default=5.0):
    """
    가짜 스캔: 기본은 사방 default [m]. obstacles 는 (로봇 기준 각도 °부터, °까지, 거리).

    스캔 각도 = 로봇 각도 + 180° (라이다가 180° 돌아 있다).
    """
    ranges = []
    for i in range(N):
        robot = math.degrees(math.atan2(math.sin(-math.pi + i * INC - math.pi),
                                        math.cos(-math.pi + i * INC - math.pi)))
        value = default
        for low, high, distance in obstacles:
            if low <= robot <= high:
                value = distance
        ranges.append(value)
    return ex.scan_points(ranges, -math.pi, INC, 0.05)


def test_open_space_goes_full_speed():
    points, blind = scan()
    assert blind == 0.0
    assert ex.drive_command(points, blind, 0.08) == (0.08, 0.0)


def test_wall_ahead_blocks_and_slows_before():
    points, blind = scan([(-20, 20, 0.25)])
    assert ex.drive_command(points, blind, 0.08) is None
    points, blind = scan([(-20, 20, 0.45)])
    linear, _ = ex.drive_command(points, blind, 0.08)
    assert 0.08 * ex.MIN_SPEED_RATIO <= linear < 0.08


def test_corner_outside_narrow_angle_still_blocks():
    """앞 ±30° 밖이어도 로봇 폭 안으로 들어오는 모서리(40° 방향 0.17 m)는 막힌 것이다."""
    points, blind = scan([(38, 42, 0.17)])
    assert ex.drive_command(points, blind, 0.08) is None


def test_far_side_object_does_not_block():
    points, blind = scan([(60, 90, 0.5)])
    linear, angular = ex.drive_command(points, blind, 0.08)
    assert linear == 0.08 and angular < 0          # 왼쪽이 가까우니 오른쪽으로 조금 튼다


def test_missing_returns_ahead_count_as_blocked():
    """앞쪽 값이 inf 로 비면(검은 벽, 유리) 가지 않는다."""
    points, blind = scan([(-25, 25, math.inf)])
    assert blind > 0.9
    assert ex.drive_command(points, blind, 0.08) is None


def test_turns_toward_wider_side():
    points, _ = scan([(-20, 20, 0.25), (20, 120, 0.4)])     # 앞과 왼쪽이 막힘
    assert ex.wider_side(points) == -1.0
    points, _ = scan([(-20, 20, 0.25), (-120, -20, 0.4)])
    assert ex.wider_side(points) == 1.0


def test_back_clearance_sees_wall_behind():
    points, _ = scan([(160, 180, 0.2), (-180, -160, 0.2)])
    _, back, _, _ = ex.clearances(points)
    assert back < 0.25
