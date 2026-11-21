"""시계 보정 중계(clock_bridge)의 시계 차이 계산 검사."""
from types import SimpleNamespace

from pinky_parking_bringup.clock_bridge import ClockOffset
from pinky_parking_bringup.clock_bridge import set_stamp
from pinky_parking_bringup.clock_bridge import stamp_ns

S = 1_000_000_000


def test_offset_is_smallest_delay_seen():
    offset = ClockOffset(window=5)
    assert offset.apply(0) is None                         # 아직 못 쟀다
    behind = 17 * 24 * 3600 * S                            # 로봇 시계가 17일 늦다
    for delay_ms in (30, 12, 8, 25):                       # 와이파이 지연이 들쭉날쭉
        offset.update(received_ns=100 * S + behind + delay_ms * 1_000_000, stamped_ns=100 * S)
    assert offset.value == behind + 8 * 1_000_000          # 가장 짧았던 지연을 쓴다
    assert offset.apply(200 * S) == 200 * S + behind + 8 * 1_000_000


def test_window_forgets_old_samples():
    offset = ClockOffset(window=2)
    offset.update(10, 0)
    offset.update(50, 0)
    offset.update(60, 0)
    assert offset.value == 50                              # 10 은 창 밖으로 나갔다


def test_stamp_round_trip():
    stamp = SimpleNamespace(sec=0, nanosec=0)
    set_stamp(stamp, 12 * S + 345)
    assert (stamp.sec, stamp.nanosec) == (12, 345) and stamp_ns(stamp) == 12 * S + 345
