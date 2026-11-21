"""마커 검출(marker_check.make_detector)과 경기장 마커 모델 검사."""
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def synthetic(marker_id, side=120, pad=40, squash=1.0):
    """흰 바탕에 마커 하나를 그린 회색 영상 (squash < 1 이면 세로로 납작하게)."""
    cv2 = pytest.importorskip('cv2')
    table = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(table, marker_id, side)
    else:
        marker = cv2.aruco.drawMarker(table, marker_id, side)
    image = cv2.copyMakeBorder(marker, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    if squash != 1.0:
        image = cv2.resize(image, None, fx=1, fy=squash, interpolation=cv2.INTER_AREA)
    return image


def test_detects_marker_id_and_center():
    from pinky_parking_gazebo.marker_check import make_detector
    found = make_detector()(synthetic(11))
    assert [marker_id for marker_id, _ in found] == [11]
    u, v = found[0][1]
    assert u == pytest.approx(100, abs=3) and v == pytest.approx(100, abs=3)


def test_detects_flat_marker_with_stretch():
    """낮은 카메라에서 납작하게 보이는 바닥 마커: 세로로 늘려 찾고, 가운데는 원래 영상 기준."""
    from pinky_parking_gazebo.marker_check import make_detector
    found = make_detector()(synthetic(20, squash=0.35))
    assert [marker_id for marker_id, _ in found] == [20]
    assert found[0][1][1] == pytest.approx(200 * 0.35 / 2, abs=4)


def test_arena_markers_have_models_and_world_includes():
    world = (ROOT / 'worlds/arena.world').read_text(encoding='utf-8')
    for marker_id in (10, 11, 20, 30):
        folder = ROOT / f'models/aruco_{marker_id}'
        assert (folder / 'model.sdf').is_file() and (folder / 'model.config').is_file()
        assert (folder / f'materials/textures/aruco_{marker_id}.png').is_file()
        assert f'<uri>model://aruco_{marker_id}</uri>' in world


def test_corner_order_is_aruco_order():
    """꼭짓점은 마커의 왼쪽 위부터 시계 방향 (마커를 똑바로 그린 영상에서)."""
    from pinky_parking_gazebo.marker_check import make_corner_detector
    corners = make_corner_detector()(synthetic(30))[30]
    (u0, v0), (u1, v1), (u2, v2), (u3, v3) = corners
    assert u0 < u1 and v1 < v2 and u2 > u3 and v0 < v3


class FakeImage:
    """sensor_msgs/Image 대신 (mono8)."""

    def __init__(self, gray):
        self.height, self.width = gray.shape
        self.step, self.encoding, self.data = self.width, 'mono8', gray.tobytes()


def test_sim_camera_reply_has_markers_and_camera():
    from pinky_parking_gazebo.sim_camera import Frames
    from pinky_parking_gazebo.sim_camera import markers_reply
    frames = Frames(forward=0.035)
    frames.set_info(985.5, 640.0, 360.0)
    frames.update(FakeImage(synthetic(10)))
    reply = markers_reply(frames)
    assert [m['id'] for m in reply['markers']] == [10]
    assert len(reply['markers'][0]['corners']) == 4
    assert reply['camera'] == {'focal_px': 985.5, 'cx': 640.0, 'cy': 360.0, 'forward': 0.035}
    assert (reply['width'], reply['height']) == (200, 200) and frames.jpeg
