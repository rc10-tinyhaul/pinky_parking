"""마커로 로봇 위치 계산(markers) 검사."""
import math

from pinky_parking_navigation import markers as geo
import pytest


def project(marker_robot, size=0.05, focal=635.0, center=(320.0, 240.0), cam_height=0.073,
            pitch=math.radians(11.0), forward=0.05):
    """바닥의 마커를 카메라 화면에 그린다 (시험용 카메라 모델). 꼭짓점 네 개의 화면 좌표."""
    mx, my, myaw = marker_robot
    half = size / 2.0
    out = []
    for lx, ly in ((-half, half), (half, half), (half, -half), (-half, -half)):
        # 마커 좌표 → 로봇 바닥 좌표 (앞, 왼쪽)
        x = mx + math.cos(myaw) * lx - math.sin(myaw) * ly - forward
        y = my + math.sin(myaw) * lx + math.cos(myaw) * ly
        # 로봇 좌표(앞 x, 왼쪽 y, 위 z) → 카메라 좌표(오른쪽, 아래, 앞), 아래로 pitch 만큼 숙임
        zc = x * math.cos(pitch) + cam_height * math.sin(pitch)
        yc = -x * math.sin(pitch) + cam_height * math.cos(pitch)
        out.append([center[0] + focal * (-y) / zc, center[1] + focal * yc / zc])
    return out


@pytest.mark.parametrize('marker', [
    (0.22, 0.0, -math.pi / 2), (0.30, 0.05, -1.2), (0.18, -0.03, 0.4)])
def test_marker_in_robot_recovers_floor_pose(marker):
    result = geo.marker_in_robot(project(marker), 0.05, 635.0, (320.0, 240.0), camera_forward=0.05)
    assert result is not None
    assert result['pose'][0] == pytest.approx(marker[0], abs=0.004)
    assert result['pose'][1] == pytest.approx(marker[1], abs=0.004)
    assert result['pose'][2] == pytest.approx(marker[2], abs=0.03)
    assert result['height'] == pytest.approx(0.073, abs=0.004)       # 카메라 높이도 맞게 나온다


def test_register_then_localize_round_trip():
    robot = (0.0, 0.15, math.pi / 2)                       # 11번 칸
    seen = (0.22, 0.014, -1.46)                            # 그 자리에서 본 마커
    marker_map = geo.marker_from_robot(robot, seen)
    assert marker_map[0] == pytest.approx(-0.014, abs=1e-6)
    assert marker_map[1] == pytest.approx(0.37)
    assert geo.difference(geo.robot_from_marker(marker_map, seen), robot) == pytest.approx((0, 0))
    # 로봇이 5 cm 앞으로 가면 마커는 그만큼 가까이 보이고, 계산한 위치도 5 cm 앞이다
    moved = geo.robot_from_marker(marker_map, (0.17, 0.014, -1.46))
    assert moved[1] == pytest.approx(0.20, abs=1e-6) and moved[0] == pytest.approx(0.0, abs=1e-6)


def test_median_ignores_one_bad_sample():
    poses = [(0.2, 0.0, 0.1), (0.21, 0.01, 0.1), (0.9, 0.5, 0.1)]
    assert geo.median_pose(poses)[:2] == (0.21, 0.01)


def test_detection_payload():
    d = geo.detection(11, (0.30, 0.0, 0.0), registered=True)
    assert d == {'id': 11, 'x': 0.3, 'y': 0.0, 'yaw': 0.0, 'distance': 0.3, 'bearing': 0.0,
                 'registered': True}
    left = geo.detection(20, (0.2, 0.2, 1.0))
    assert left['bearing'] == pytest.approx(math.pi / 4, abs=1e-4) and not left['registered']


def test_camera_from_reply_uses_server_values_when_given():
    defaults = (635.0, (320.0, 240.0), 0.05)
    assert geo.camera_from_reply({'markers': []}, *defaults) == defaults     # 실물 서버
    sim = {'camera': {'focal_px': 985.5, 'cx': 640.0, 'cy': 360.0, 'forward': 0.035}}
    assert geo.camera_from_reply(sim, *defaults) == (985.5, (640.0, 360.0), 0.035)


def test_sim_camera_values_recover_pose():
    """시뮬레이션 카메라(1280 x 720, 화각 66°)의 값으로도 바닥 마커 위치가 맞게 나온다."""
    focal, center = 640.0 / math.tan(1.1519 / 2), (640.0, 360.0)
    corners = project((0.30, 0.02, 0.1), focal=focal, center=center, cam_height=0.063,
                      pitch=math.radians(8.0), forward=0.035)
    result = geo.marker_in_robot(corners, 0.05, focal, center, camera_forward=0.035)
    assert result['pose'][0] == pytest.approx(0.30, abs=0.004)
    assert result['pose'][1] == pytest.approx(0.02, abs=0.004)


def test_arena_markers_match_arena_layout():
    """markers.yaml 의 경기장 마커(10, 11, 20, 30)는 경기장 원본(arena.yaml)과 같은 자리다."""
    from pathlib import Path
    import yaml
    root = Path(__file__).resolve().parents[2]
    arena_file = root / 'pinky_parking_control/config/arena.yaml'
    if not arena_file.is_file():
        pytest.skip('경기장 원본이 없습니다')
    arena = yaml.safe_load(arena_file.read_text(encoding='utf-8'))['markers']['items']
    table = yaml.safe_load((root / 'pinky_parking_navigation/config/markers.yaml')
                           .read_text(encoding='utf-8'))['markers']
    for marker_id, m in arena.items():
        assert table[marker_id] == {'x': m['x'], 'y': m['y'], 'yaw': m['yaw']}


def test_fully_visible_rejects_markers_touching_the_edge():
    inside = [[600, 600], [680, 600], [690, 650], [590, 650]]
    assert geo.fully_visible(inside, 1280, 720)
    cut = [[600, 690], [680, 690], [690, 719], [590, 719]]         # 아래 가장자리에 걸림
    assert not geo.fully_visible(cut, 1280, 720)
    assert geo.fully_visible(cut, 0, 0)                              # 화면 크기를 모르면 거르지 않음
