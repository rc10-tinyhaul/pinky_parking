"""SLAM 지도를 map 좌표로 돌려 저장하기(save_slam_map) 검사."""
import math

from pinky_parking_navigation.save_slam_map import FREE
from pinky_parking_navigation.save_slam_map import OCCUPIED
from pinky_parking_navigation.save_slam_map import to_map_frame
from pinky_parking_navigation.save_slam_map import UNKNOWN
from pinky_parking_navigation.save_slam_map import write_map


def test_identity_keeps_grid_and_flips_rows_for_pgm():
    # 2 x 2 격자, 아랫줄부터: [빈, 장애물], [모름, 빈]
    w, h, origin, px = to_map_frame(2, 2, 0.5, (0.0, 0.0), [0, 100, -1, 0], (0.0, 0.0, 0.0))
    assert (w, h, origin) == (2, 2, (0.0, 0.0))
    assert list(px) == [UNKNOWN, FREE, FREE, OCCUPIED]            # 윗줄 먼저


def test_start_pose_rotates_and_shifts():
    # slam 프레임에서 로봇 앞(+x) 1 m 에 벽 한 칸. 로봇은 지도에서 (0, 0.15) 에 위(+y)를 보고 시작
    data = [0] * 9
    data[1 * 3 + 2] = 100                                          # 칸 (2, 1): 중심 slam (1.0, 0.0)
    w, h, origin, px = to_map_frame(3, 3, 0.5, (-0.25, -0.75), data, (0.0, 0.15, math.pi / 2))
    cells = [(col, row) for row in range(h) for col in range(w)
             if px[(h - 1 - row) * w + col] == OCCUPIED]
    assert len(cells) == 1
    col, row = cells[0]
    x, y = origin[0] + (col + 0.5) * 0.5, origin[1] + (row + 0.5) * 0.5
    assert abs(x - 0.0) <= 0.25 and abs(y - 1.15) <= 0.25          # 지도에서는 (0, 1.15): 위쪽 1 m


def test_write_map_files(tmp_path):
    path = write_map(tmp_path / 'maps/room', 2, 1, 0.05, (-1.0, 2.0), bytes([FREE, OCCUPIED]),
                     note='시험')
    text = path.read_text(encoding='utf-8')
    assert 'image: room.pgm' in text and 'origin: [-1.000, 2.000, 0]' in text
    assert (tmp_path / 'maps/room.pgm').read_bytes() == b'P5\n2 1\n255\n' + bytes([FREE, OCCUPIED])
