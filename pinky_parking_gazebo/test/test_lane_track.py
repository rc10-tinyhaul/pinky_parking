"""차선 시험 트랙(lane_track.py) 검사."""
import math
from pathlib import Path

from pinky_parking_gazebo.lane_track import lane_offset
from pinky_parking_gazebo.lane_track import load_track
from pinky_parking_gazebo.lane_track import tape_boxes
from pinky_parking_gazebo.lane_track import tape_radii
from pinky_parking_gazebo.lane_track import world_sdf

PACKAGE = Path(__file__).resolve().parents[1]
TRACK = load_track(PACKAGE / 'config/lane_track.yaml')


def test_tapes_between_and_around_lanes():
    # 2차선: 안쪽, 가운데(공유), 바깥쪽 테이프 3줄
    assert tape_radii(TRACK) == [0.31, 0.5, 0.69]


def test_lane_offset_on_straight_and_curve():
    assert lane_offset(TRACK, 0.405, 0.0) == (1, 0.0)
    lane, offset = lane_offset(TRACK, 0.0, 0.3 + 0.62)   # 위쪽 곡선 꼭대기, 2번 차선 바깥쪽 25 mm
    assert lane == 2 and abs(offset - 0.025) < 1e-9
    lane, offset = lane_offset(TRACK, 0.45, 0.0, lane=1)  # 차선을 정하면 그 차선 기준
    assert lane == 1 and abs(offset - 0.045) < 1e-9


def test_tape_pieces_lie_on_tape_lines():
    for x, y, _, _ in tape_boxes(TRACK):
        distance = math.hypot(x, y - max(-0.3, min(0.3, y)))
        assert min(abs(distance - r) for r in tape_radii(TRACK)) < 0.001


def test_world_file_is_up_to_date():
    # worlds/lane_test.sdf 는 make_lane_world 로 만든 파일. 치수를 바꾸면 다시 만들어야 함
    assert (PACKAGE / 'worlds/lane_test.sdf').read_text() == world_sdf(TRACK)
