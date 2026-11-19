"""track_check 의 차선 변경 찾기(find_changes) 검사. 합성 경로를 넣는다."""
import math
from pathlib import Path

from pinky_parking_gazebo.lane_track import load_track
from pinky_parking_gazebo.track_check import find_changes

TRACK = load_track(Path(__file__).resolve().parents[1] / 'config/lane_track.yaml')


def samples_from(points):
    out, travelled = [], 0.0
    for i, (x, y) in enumerate(points):
        if i:
            travelled += math.hypot(x - points[i - 1][0], y - points[i - 1][1])
        out.append((x, y, travelled))
    return out


def right_straight(x_of_y, y_from=-0.3, y_to=0.3, step=0.002):
    """오른쪽 직선(+y 방향)을 따라가는 점들. x_of_y(y) = 그 y 에서의 x."""
    n = int((y_to - y_from) / step)
    return [(x_of_y(y_from + i * step), y_from + i * step) for i in range(n + 1)]


def smooth(a, b, y0, y1):
    """y0~y1 사이에서 x 가 a 에서 b 로 부드럽게 바뀜."""
    def x(y):
        u = min(1.0, max(0.0, (y - y0) / (y1 - y0)))
        return a + (b - a) * (3 * u * u - 2 * u * u * u)
    return x


def test_change_on_straight():
    path = right_straight(smooth(0.405, 0.595, -0.25, 0.20))
    changes = find_changes(TRACK, samples_from(path))
    assert len(changes) == 1
    change = changes[0]
    assert (change['from'], change['to']) == (1, 2)
    assert change['on_straight']
    assert -0.3 <= change['start_y'] < change['end_y'] <= 0.3
    assert change['overshoot'] < 0.001


def test_overshoot_is_measured():
    def x(y):   # 바깥으로 10 mm 넘어간 뒤 돌아옴
        if y < 0.10:
            return smooth(0.405, 0.605, -0.25, 0.10)(y)
        return 0.605 - 0.01 * (y - 0.10) / 0.2
    changes = find_changes(TRACK, samples_from(right_straight(x)))
    assert len(changes) == 1 and abs(changes[0]['overshoot'] - 0.010) < 0.001


def test_change_past_the_straight_is_flagged():
    path = right_straight(smooth(0.405, 0.595, 0.0, 0.5), y_to=0.6)
    changes = find_changes(TRACK, samples_from(path))
    assert len(changes) == 1 and not changes[0]['on_straight']


def test_brief_touch_of_other_lane_is_not_a_change():
    def x(y):
        return 0.405 + (0.1 if 0.0 <= y <= 0.02 else 0.0)  # 2 cm 동안만 2번 차선 쪽
    assert find_changes(TRACK, samples_from(right_straight(x))) == []
