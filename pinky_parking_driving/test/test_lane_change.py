"""차선 변경(LaneChanger) 검사: 상태 전이와, 카메라 시야를 흉내 낸 닫힌 루프 주행."""
import math

import numpy as np
from pinky_parking_driving.lane import Camera
from pinky_parking_driving.lane import find_lane
from pinky_parking_driving.lane import LaneChanger
from pinky_parking_driving.lane import LaneResult
from pinky_parking_driving.lane import pursuit_curvature

WIDTH = 0.19
TAPE = 0.02
CAMERA = Camera(height=0.063, x=0.033, tilt=math.radians(8), hfov=1.1519)
IMAGE = (1280, 720)
FIND = {'lane_width': WIDTH, 'near': 0.17, 'far': 0.26, 'step': 0.02}


def straight_result(offset=0.0):
    return LaneResult(True, target=(0.25, offset), near_offset=offset, slices=5, offset=offset)


def drive(changer, frames, curvature, distance, offset=0.0, step=0.003):
    """같은 영상 결과를 frames 번 넣고, 나온 상태 문자열과 누적 거리를 돌려준다."""
    events = []
    for _ in range(frames):
        distance += step
        event = changer.update(straight_result(offset), curvature, distance)
        if event:
            events.append(event)
    return events, distance


def test_request_rules():
    assert LaneChanger(WIDTH).request(2) == 'failed:2:no_lane'
    changer = LaneChanger(WIDTH, lane=1)
    assert changer.request(3) == 'failed:3:invalid'
    assert changer.request(1) == 'done:1'
    assert changer.request(2) == 'waiting:2'
    assert changer.request(2) is None              # 같은 요청이 다시 와도 그대로


def test_starts_only_where_a_straight_begins():
    changer = LaneChanger(WIDTH, lane=1)
    changer.request(2)
    events, d = drive(changer, 20, 0.1, 0.0)        # 직선 중간에서 요청: 기다림
    assert events == [] and changer.state == 'waiting'
    events, d = drive(changer, 20, 2.5, d)          # 곡선
    assert events == []
    events, d = drive(changer, 5, 0.1, d)           # 직선이 시작됨
    assert events == ['changing:2']
    assert changer.start_y() == -WIDTH              # 오른쪽 차선 중심에서 찾음


def test_late_straight_is_skipped():
    # 직선이 시작된 곳(판단 기준)에서 max_start_distance 를 넘기면 이번 직선은 건너뜀
    changer = LaneChanger(WIDTH, lane=1, max_start_distance=0.10, start_after=0.15)
    changer.request(2)
    _, d = drive(changer, 20, 2.5, 0.0)
    _, d = drive(changer, 5, 0.1, d)               # 직선 시작, 아직 start_after 전
    assert changer.state == 'waiting'
    changer.update(straight_result(), 0.1, d + 0.2)  # 0.2 m 가 지나 늦음
    assert changer.state == 'waiting'
    _, d = drive(changer, 20, 0.1, d + 0.2)        # 같은 직선에서는 다시 시작하지 않음
    assert changer.state == 'waiting'


def test_done_and_failures():
    changer = LaneChanger(WIDTH, lane=1)
    changer.request(2)
    _, d = drive(changer, 20, 2.5, 0.0)
    _, d = drive(changer, 5, 0.1, d)
    assert changer.state == 'changing'
    events, d = drive(changer, 5, 0.1, d, offset=0.005)
    assert events == ['done:2'] and changer.lane == 2 and changer.start_y() == 0.0

    changer.request(1)
    _, d = drive(changer, 20, 2.5, d)
    _, d = drive(changer, 5, 0.1, d)
    events, d = drive(changer, 400, 0.1, d, offset=0.08)   # 중심에 오지 못함
    assert events == ['failed:1:too_long'] and changer.lane is None

    # 거리 제한에 닿음(곡선에 들어감 등): 목표 차선 안이면 넘기고, 아니면 실패
    changer = LaneChanger(WIDTH, lane=2)
    changer.request(1)
    _, d = drive(changer, 20, 2.5, 0.0)
    _, d = drive(changer, 5, 0.1, d)
    events, d = drive(changer, 400, 2.0, d, offset=0.03)
    assert events == ['done:1'] and changer.lane == 1

    changer = LaneChanger(WIDTH, lane=1, wait_distance=1.0)
    changer.request(2)
    events, _ = drive(changer, 400, 0.1, 0.0)               # 곡선→직선이 없음
    assert events == ['failed:2:no_straight'] and changer.lane == 1


def tapes_in_view(pose, tape_ys):
    """세계 좌표의 직선 테이프(y = 값, x 방향)를 로봇 좌표로 옮겨 카메라에 보이는 점만 남긴다."""
    x0, y0, heading = pose
    xs, ys = [], []
    for along in np.arange(x0 - 0.2, x0 + 0.8, 0.004):
        for tape_y in tape_ys:
            for w in np.linspace(-TAPE / 2, TAPE / 2, 5):
                dx, dy = along - x0, tape_y + w - y0
                xs.append(math.cos(heading) * dx + math.sin(heading) * dy)
                ys.append(-math.sin(heading) * dx + math.cos(heading) * dy)
    xs, ys = np.array(xs), np.array(ys)
    c, s = math.cos(CAMERA.tilt), math.sin(CAMERA.tilt)
    f = (IMAGE[0] / 2) / math.tan(CAMERA.hfov / 2)
    forward = c * (xs - CAMERA.x) + s * CAMERA.height
    with np.errstate(divide='ignore', invalid='ignore'):
        u = IMAGE[0] / 2 - f * ys / forward
        v = IMAGE[1] / 2 - f * (s * (xs - CAMERA.x) - c * CAMERA.height) / forward
    keep = (forward > 0) & (u >= 0) & (u < IMAGE[0]) & (v >= 0) & (v < IMAGE[1]) & (xs < 0.5)
    return xs[keep], ys[keep]


def test_closed_loop_change_to_the_right():
    # 차선 1(중심 y = 0) → 차선 2(중심 y = -0.19). 테이프 3줄, 카메라에 보이는 것만 씀.
    tapes = (WIDTH / 2, -WIDTH / 2, -1.5 * WIDTH)
    changer = LaneChanger(WIDTH, lane=1)
    changer.request(2)
    changer._in_curve = True                          # 곡선을 막 빠져나온 상태
    x, y, heading, travelled, dt = 0.0, 0.0, 0.0, 0.0, 1 / 30
    started = None
    path = []
    for _ in range(600):
        xs, ys = tapes_in_view((x, y, heading), tapes)
        result = find_lane(xs, ys, start=changer.start_y(), **FIND)
        assert result.found, f'차선을 놓침: y={y:.3f}, heading={math.degrees(heading):.0f}°'
        curvature = pursuit_curvature(result.target)
        event = changer.update(result, curvature, travelled)
        if event == 'changing:2':
            started = travelled
        if event == 'done:2':
            break
        speed = 0.10 / (1 + 0.15 * abs(curvature))
        omega = max(-1.5, min(1.5, speed * curvature))
        x += speed * math.cos(heading) * dt
        y += speed * math.sin(heading) * dt
        heading += omega * dt
        travelled += speed * dt
        if started is not None:
            path.append((travelled - started, y, heading))
    assert changer.lane == 2
    within = next(s for s, py, _ in path if abs(py + WIDTH) < 0.02)
    overshoot = max(0.0, -min(py for _, py, _ in path) - WIDTH)
    peak = max(abs(h) for _, _, h in path)
    # 직선은 0.6 m 이고, 변경은 직선이 시작되고 조금 뒤(최대 약 0.05 m)에 시작한다
    assert within <= 0.55, f'중심 20 mm 안까지 {within:.2f} m'
    assert overshoot <= 0.015, f'넘어간 거리 {overshoot * 1000:.0f} mm'
    assert peak <= math.radians(30), f'최대 방향 {math.degrees(peak):.0f}°'


def test_config_matches_lane_changer():
    # config/lane.yaml 의 change: 항목 이름이 LaneChanger 인자와 맞아야 lane_follow 가 뜸
    from pathlib import Path

    import yaml
    config = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/lane.yaml').read_text())
    LaneChanger(config['lane']['width'], lane=1, **config['change'])
