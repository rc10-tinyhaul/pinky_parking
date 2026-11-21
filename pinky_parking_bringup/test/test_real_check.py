"""실물 연결 점검(real_check.judge) 검사."""
from pinky_parking_bringup.real_check import BAD
from pinky_parking_bringup.real_check import judge
from pinky_parking_bringup.real_check import OK
from pinky_parking_bringup.real_check import WARN


def marks(rows):
    return {item: mark for mark, item, _ in rows}


def good(**changes):
    values = {'odom_count': 150, 'scan_count': 50, 'seconds': 5.0, 'odom_publishers': 1,
              'scan_publishers': 1, 'clock_offset': 0.02, 'watchdog': True,
              'cmd_vel_subscribers': 2, 'battery_v': 7.8}
    values.update(changes)
    return judge(**values)


def test_healthy_robot_passes():
    assert set(marks(good()).values()) == {OK}


def test_no_robot_and_no_watchdog_fail():
    result = marks(good(odom_count=0, scan_count=0, odom_publishers=0, scan_publishers=0,
                        clock_offset=None, watchdog=False, cmd_vel_subscribers=0,
                        battery_v=None))
    assert result['오도메트리 /odom'] == BAD and result['정지 감시 노드'] == BAD
    assert result['구동 /cmd_vel'] == BAD and result['배터리'] == WARN


def test_other_robot_on_same_domain_is_caught():
    assert marks(good(odom_publishers=2))['도메인 겹침'] == BAD


def test_clock_and_battery_thresholds():
    assert marks(good(clock_offset=-0.3))['시계 차이'] == WARN
    assert marks(good(clock_offset=2.0))['시계 차이'] == BAD
    assert marks(good(battery_v=6.9))['배터리'] == BAD


def test_bridged_accepts_any_clock_offset():
    rows = good(clock_offset=-1530687.0, bridged=True)
    assert marks(rows)['시계 차이'] == OK
