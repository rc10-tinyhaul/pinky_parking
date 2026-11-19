"""속도 명령 감시(Watchdog) 검사."""
from pinky_parking_bringup.cmd_vel_watchdog import Watchdog


def test_stops_once_after_silence():
    w = Watchdog(timeout=0.5)
    w.on_command(0.1, 0.0, now=0.0)
    assert not w.should_stop(0.4)
    assert w.should_stop(0.6)          # 0.5 s 넘게 끊김
    assert not w.should_stop(0.7)      # 한 번만


def test_steady_commands_keep_moving():
    w = Watchdog(timeout=0.5)
    for i in range(20):
        w.on_command(0.1, 0.2, now=i * 0.1)
        assert not w.should_stop(i * 0.1 + 0.05)


def test_zero_command_ends_watch():
    # 보내는 쪽이 정지 명령을 보내고 조용해지는 것은 정상 (Nav2 목표 도착 등). 자기가 낸 0 도 같음
    w = Watchdog(timeout=0.5)
    w.on_command(0.1, 0.0, now=0.0)
    w.on_command(0.0, 0.0, now=0.2)
    assert not w.should_stop(5.0)


def test_rotation_only_counts_as_moving():
    w = Watchdog(timeout=0.5)
    w.on_command(0.0, 0.5, now=0.0)
    assert w.should_stop(1.0)
