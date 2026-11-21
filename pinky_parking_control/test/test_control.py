"""관제 API 의 순수 로직 시험 (ROS 없이)."""
import math
from pathlib import Path
import time

from pinky_parking_control.fleet import Fleet
from pinky_parking_control.fleet import UnknownName
from pinky_parking_control.layout import ArenaLayout
from pinky_parking_control.layout import choose_layout
from pinky_parking_control.layout import FactoryLayout
from pinky_parking_control.layout import NotInLayout
from pinky_parking_control.layout import Stadium
from pinky_parking_control.mapdata import classify
from pinky_parking_control.mapdata import load_map
from pinky_parking_control.mapdata import read_pgm
from pinky_parking_control.model import CANCELED
from pinky_parking_control.model import NotYet
from pinky_parking_control.model import parse_pose
from pinky_parking_control.model import quaternion_to_yaw
from pinky_parking_control.model import RUNNING
from pinky_parking_control.model import SUCCEEDED
from pinky_parking_control.model import Task
from pinky_parking_control.model import TaskBusy
from pinky_parking_control.model import TaskLog
from pinky_parking_control.model import yaw_to_quaternion
from pinky_parking_control.scenario import ScenarioRunner
import pytest
import yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
ROBOTS = [{'name': 'pinky1', 'domain_id': 11}, {'name': 'pinky2', 'domain_id': 12}]
SLOTS = {'slot_10': {'x': 0.3, 'y': -1.1, 'yaw': 0.0}}
ROUTES = {'lap': [{'x': -0.8, 'y': 1.15, 'yaw': 3.14159}, {'x': 0.5, 'y': 1.0, 'yaw': 0.0}]}
CONFIG = {'timeout_s': {'goto': 5}, 'vehicles': {'pinky2': 'T10'},
          'teleop': {'max_linear': 0.1, 'max_angular': 1.0}}
CALLSIGNS = {'A': 'pinky1', 'B': 'pinky2'}


def read_yaml(name):
    return yaml.safe_load((CONFIG_DIR / name).read_text(encoding='utf-8'))


class FakeLink:
    """RobotLink 대신: Nav2 없이 작업을 받는다. auto=True 면 받자마자 성공한다."""

    auto = False

    def __init__(self, name, domain_id, on_finish):
        self.name, self.domain_id, self.on_finish = name, domain_id, on_finish
        self.task, self.stopped, self.closed = None, False, False
        self.pose = {'x': 0.405, 'y': 0.0, 'yaw': math.pi / 2}
        self.twist = None

    def start(self, task):
        if self.task is not None and not self.task.done:
            raise TaskBusy('busy')
        self.task = task
        if self.auto:
            self.pose = dict(task.poses[-1])
            task.finish(SUCCEEDED)
            self.on_finish(task)
        return task

    def cancel(self, reason='취소했습니다'):
        if self.task is None or self.task.done:
            return False
        self.task.finish(CANCELED, reason)
        self.on_finish(self.task)
        return True

    def stop(self):
        self.stopped = True
        return self.cancel('정지 명령')

    def teleop(self, linear, angular):
        if self.task is not None and not self.task.done:
            raise TaskBusy('busy')
        self.twist = (linear, angular)
        return {'linear': linear, 'angular': angular}

    def teleop_watchdog(self, now, timeout_s):
        pass

    def status(self):
        return {'name': self.name, 'pose': self.pose,
                'task': self.task.to_dict() if self.task else None}

    def close(self):
        self.closed = True


class AutoLink(FakeLink):
    auto = True


@pytest.fixture
def factory():
    layout = FactoryLayout(SLOTS, ROUTES, {'slot_10': {'x': 0, 'y': 0}},
                           read_yaml('arena.yaml')['vehicles'])
    f = Fleet(ROBOTS, layout, CONFIG, FakeLink, CALLSIGNS)
    yield f
    f.close()


@pytest.fixture
def arena():
    f = Fleet(ROBOTS, ArenaLayout(read_yaml('arena.yaml')), CONFIG, FakeLink, CALLSIGNS)
    yield f
    f.close()


# ---- 모델 -------------------------------------------------------------------
def test_quaternion_round_trip():
    for yaw in (-3.0, -1.2, 0.0, 0.7, 3.1):
        assert math.isclose(quaternion_to_yaw(*yaw_to_quaternion(yaw)), yaw, abs_tol=1e-9)


def test_parse_pose_checks_numbers():
    assert parse_pose({'x': '1', 'y': 2}) == {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    assert math.isclose(parse_pose({'x': 0, 'y': 0, 'yaw': 2 * math.pi + 0.5})['yaw'], 0.5)
    for bad in ({'x': 1}, {'x': 'a', 'y': 0}, {'x': float('nan'), 'y': 0}, None):
        with pytest.raises(ValueError):
            parse_pose(bad)


def test_task_finishes_once():
    task = Task(robot='pinky1', action='goto', target='t', poses=[{'x': 0, 'y': 0, 'yaw': 0}],
                timeout_s=10, started=100.0)
    assert task.state == RUNNING and not task.timed_out(now=105.0) and task.timed_out(now=111.0)
    assert task.finish(SUCCEEDED, now=104.0)
    assert not task.finish(CANCELED, now=105.0)   # 먼저 온 결과가 이긴다
    assert task.state == SUCCEEDED and task.elapsed() == 4.0
    with pytest.raises(ValueError):
        Task(robot='pinky1', action='fly', target='t', poses=[{}], timeout_s=1)


def test_task_log_keeps_newest_first():
    log = TaskLog(size=2)
    for i in range(3):
        t = Task(robot='pinky1', action='goto', target=str(i), poses=[{'x': 0, 'y': 0, 'yaw': 0}],
                 timeout_s=1)
        t.finish(SUCCEEDED)
        log.add(t)
    assert [t['target'] for t in log.recent()] == ['2', '1']


# ---- 경기장 도면 ------------------------------------------------------------
def test_stadium_follows_drawing():
    track = Stadium(0.405, 0.6)
    assert math.isclose(track.length, 1.2 + 2 * math.pi * 0.405)
    x, y, yaw = track.point(0.3)                          # 오른쪽 직선 가운데, 위로 (반시계)
    assert math.isclose(x, 0.405) and abs(y) < 1e-9 and math.isclose(yaw, math.pi / 2)
    x, y, yaw = track.point(0.6 + math.pi * 0.405 / 2)    # 위 반원 꼭대기, 왼쪽으로
    assert math.isclose(y, 0.3 + 0.405) and abs(x) < 1e-9 and math.isclose(abs(yaw), math.pi)
    s, d = track.nearest(-0.405, 0.0)                     # 왼쪽 직선 가운데
    assert d < 0.01 and math.isclose(s, 0.6 + math.pi * 0.405 + 0.3, abs_tol=0.01)


def test_arena_fits_board_like_drawing():
    """도면(docs/arena_map_2000x2000.jpg, 1 px = 1 mm)처럼 모든 것이 바닥 안에 1:1 로 들어간다."""
    data = read_yaml('arena.yaml')
    b = data['board']
    x0, y0, x1, y1 = b['x'], b['y'], b['x'] + b['width'], b['y'] + b['height']
    assert (b['width'], b['height']) == (2.0, 2.0)
    # 도면에서 원점(트랙 중심)은 그림 왼쪽에서 1200 mm, 위에서 1000 mm
    assert math.isclose(-x0, 1.2) and math.isclose(y1, 1.0)
    # 바깥 트랙 테이프 바깥선
    outer = max(t['radius'] for t in data['tracks']) + data['lane_width'] / 2
    top = data['straight'] / 2 + outer
    assert x0 < -outer and outer < x1 and y0 <= -top and top <= y1
    # 주차 칸, 주유구역, 대기선 (세로로 놓인 칸: 가로 = width, 세로 = length)
    zones = list(data['slots'].values()) + [data['fuel']['zone']]
    for z in zones:
        assert x0 <= z['x'] - z['width'] / 2 and z['x'] + z['width'] / 2 <= x1
        assert y0 <= z['y'] - z['length'] / 2 and z['y'] + z['length'] / 2 <= y1
    w = data['fuel']['wait_line']
    assert x0 <= w['x'] - w['width'] / 2 and w['x'] + w['width'] / 2 <= x1
    # 차종: 전체 길이 = 몸체 140 mm + 트레일러
    for v in data['vehicles'].values():
        assert math.isclose(v['length'], 0.14 + v['trailer'])


def test_arena_actions_make_waypoints(arena):
    lap = arena.drive('pinky1', 1)                         # 이어진 경로 하나 (follow_path)
    track = Stadium(0.405, 0.6)
    assert lap.target == '1번 트랙' and lap.mode == 'path' and len(lap.poses) > 100
    assert all(track.nearest(p['x'], p['y'])[1] < 0.01 for p in lap.poses)
    steps = [math.hypot(b['x'] - a['x'], b['y'] - a['y'])
             for a, b in zip(lap.poses, lap.poses[1:])]
    assert max(steps) < 0.04                               # 끊긴 데 없이 촘촘하다
    start, end = lap.poses[0], lap.poses[-1]               # 출발점 조금 앞에서 끝난다 (끝 ≠ 시작)
    assert 0.1 < math.hypot(end['x'] - start['x'], end['y'] - start['y']) < 0.2
    arena.cancel('pinky1')
    change = arena.lane_change('pinky1', 2)                # 1번 → 2번 트랙으로 매끄럽게
    assert change.mode == 'path'
    assert track.nearest(change.poses[0]['x'], change.poses[0]['y'])[1] < 0.01
    assert Stadium(0.595, 0.6).nearest(change.poses[-1]['x'], change.poses[-1]['y'])[1] < 0.01
    turns = [abs(math.atan2(math.sin(b['yaw'] - a['yaw']), math.cos(b['yaw'] - a['yaw'])))
             for a, b in zip(change.poses, change.poses[1:])]
    assert max(turns) < 0.2                                # 꺾이는 곳 없이 방향이 조금씩 바뀐다
    arena.cancel('pinky1')
    assert arena.fuel('pinky1', 'wait').poses[0]['y'] == pytest.approx(0.17)
    arena.cancel('pinky1')
    park = arena.park('pinky1', 'slot_11')     # 10번 칸을 지나지 않고 위(열린 쪽)에서 들어온다
    assert [round(p['y'], 3) for p in park.poses] == [0.43, 0.15]
    assert all(p['x'] == 0.0 and p['yaw'] == pytest.approx(-math.pi / 2, abs=1e-3)
               for p in park.poses)
    with pytest.raises(ValueError):
        arena.fuel('pinky2', 'jump')
    with pytest.raises(NotInLayout):
        arena.drive('pinky2', 3)


def test_vehicle_and_callsigns(arena):
    assert arena.robot('pinky1')['vehicle']['type'] == 'T00'
    assert arena.robot('pinky2')['vehicle']['length'] == pytest.approx(0.24)
    assert arena.set_vehicle('pinky1', 'T15')['length'] == pytest.approx(0.29)
    assert arena.robot('pinky1')['callsign'] == 'A'
    with pytest.raises(UnknownName):
        arena.set_vehicle('pinky1', 'T99')


def test_teleop_is_limited_and_blocked_while_busy(arena):
    assert arena.teleop('pinky1', 5.0, -9.0) == {'linear': 0.1, 'angular': -1.0}
    arena.park('pinky1', 'slot_10')
    with pytest.raises(TaskBusy):
        arena.teleop('pinky1', 0.1, 0.0)


def test_pending_features_say_not_yet(arena):
    for feature in ('gap', 'park_precise'):
        with pytest.raises(NotYet):
            arena.pending(feature)


def test_choose_layout():
    assert choose_layout('auto', 'pinky_factory.world') == 'factory'
    assert choose_layout('auto', 'pinky_arena.world') == 'arena'
    assert choose_layout('arena', 'pinky_factory.world') == 'arena'


# ---- 공장 월드 --------------------------------------------------------------
def test_factory_actions(factory):
    park = factory.park('pinky1', 'slot_10')
    assert park.action == 'park' and park.poses == [{'x': 0.3, 'y': -1.1, 'yaw': 0.0}]
    with pytest.raises(TaskBusy):
        factory.goto('pinky1', {'x': 0, 'y': 0})
    assert len(factory.lap('pinky2', 'lap').poses) == 2
    assert factory.cancel('pinky1') is True
    assert factory.log.recent()[0]['state'] == CANCELED
    assert factory.goto('pinky1', {'x': 1, 'y': 2}).timeout_s == 5.0
    assert factory.stop_all() == {'pinky1': True, 'pinky2': True}
    assert factory.unpark('pinky1', 'slot_10').poses == [{'x': 0.0, 'y': 0.0, 'yaw': 0.0}]
    with pytest.raises(NotInLayout):
        factory.drive('pinky2', 1)
    with pytest.raises(NotInLayout):
        factory.fuel('pinky2', 'wait')


def test_unknown_names(factory):
    for call, args in ((factory.park, ('pinky9', 'slot_10')), (factory.park, ('pinky1', 'x')),
                       (factory.lap, ('pinky1', 'nowhere')), (factory.unpark, ('pinky1', 'y'))):
        with pytest.raises(UnknownName):
            call(*args)


def test_snapshot_shape(factory):
    factory.park('pinky1', 'slot_10')
    snap = factory.snapshot()
    assert snap['layout'] == 'factory'
    assert [r['name'] for r in snap['robots']] == ['pinky1', 'pinky2']
    assert snap['robots'][0]['task']['goal'] == {'x': 0.3, 'y': -1.1, 'yaw': 0.0}
    assert snap['robots'][1]['task'] is None


# ---- 시연 시나리오 ----------------------------------------------------------
def test_scenario_runs_plan_in_order():
    fleet = Fleet(ROBOTS, ArenaLayout(read_yaml('arena.yaml')), CONFIG, AutoLink, CALLSIGNS)
    runner = ScenarioRunner(fleet, read_yaml('scenario.yaml'), poll_s=0.01)
    try:
        runner.start(run_all=True)
        for _ in range(100):                              # 사람 확인 단계에서 기다린다
            runner.wait(0.05)
            if runner.waiting:
                break
        assert runner.waiting
        states = [s['state'] for s in runner.status()['steps']]
        assert states[:5] == ['done', 'done', 'skipped', 'done', 'done']
        assert '#12' in runner.status()['steps'][2]['note']
        runner.confirm()
        runner.wait(5)
        assert all(s['state'] in ('done', 'skipped') for s in runner.status()['steps'])
        assert fleet.vehicle['pinky1'] == 'T15'
        with pytest.raises(ValueError):
            runner.start()                                 # 남은 단계 없음
        runner.reset()
        assert runner.status()['steps'][0]['state'] == 'ready'
    finally:
        runner.stop()
        fleet.close()


def test_scenario_skips_robot_that_is_not_connected():
    """한 대만 연결해도(A) 시나리오가 끝까지 간다: B 의 행동과 B 만 있는 단계는 건너뛴다."""
    fleet = Fleet(ROBOTS[:1], ArenaLayout(read_yaml('arena.yaml')), CONFIG, AutoLink, CALLSIGNS)
    runner = ScenarioRunner(fleet, read_yaml('scenario.yaml'), poll_s=0.01)
    try:
        runner.start(run_all=True)
        for _ in range(100):
            runner.wait(0.05)
            if runner.waiting:
                break
        runner.confirm()
        runner.wait(5)
        steps = runner.status()['steps']
        assert all(s['state'] in ('done', 'skipped') for s in steps)
        assert steps[0]['state'] == 'done' and 'B 없음' in steps[0]['note']      # A 는 하고 B 만 빠짐
        assert steps[4]['state'] == 'skipped' and 'B 없음' in steps[4]['note']   # B 만 있는 단계
        assert steps[8]['state'] == 'done' and steps[8]['note'] == ''
        assert [t['robot'] for t in fleet.log.recent(50)].count('pinky2') == 0
    finally:
        runner.stop()
        fleet.close()


def test_trajectory_file_is_saved(tmp_path):
    """작업이 끝나면 계획 경로와 실제 궤적을 파일로 남긴다."""
    import json
    from pinky_parking_control.model import save_trajectory
    task = Task(robot='pinky1', action='drive', target='1번 트랙', mode='path', timeout_s=10,
                poses=[{'x': 0.0, 'y': 0.0, 'yaw': 0.0}, {'x': 0.1, 'y': 0.0, 'yaw': 0.0}])
    task.trail.append([0.1, 0.0, 0.0, 0.0, 0.1, 0.0])
    task.finish(SUCCEEDED)
    data = json.loads(save_trajectory(task, tmp_path / 'trajectories').read_text())
    assert data['action'] == 'drive' and data['state'] == 'succeeded'
    assert len(data['planned']) == 2 and data['actual'] == [[0.1, 0.0, 0.0, 0.0, 0.1, 0.0]]
    assert data['actual_columns'] == ['t', 'x', 'y', 'yaw', 'v', 'w']


# ---- 지도 -------------------------------------------------------------------
def test_pgm_and_classify():
    data = b'P5\n# comment\n3 1\n255\n' + bytes([0, 254, 205])
    width, height, maxval, pixels = read_pgm(data)
    assert (width, height, maxval) == (3, 1, 255)
    assert classify(pixels, maxval, 0, 0.65, 0.196) == 'ofu'


def test_load_repository_map():
    path = Path(__file__).resolve().parents[2] / 'pinky_parking_navigation/maps/factory.yaml'
    if not path.is_file():
        pytest.skip('지도 파일이 없습니다')
    m = load_map(path)
    assert len(m['cells']) == m['width'] * m['height']
    assert set(m['cells']) <= set('ofu')


# ---- HTTP API ---------------------------------------------------------------
def client_for(fleet, **kwargs):
    pytest.importorskip('fastapi')
    pytest.importorskip('httpx')
    from fastapi.testclient import TestClient
    from pinky_parking_control.server import create_app
    return TestClient(create_app(fleet, **kwargs))


def test_http_api(factory):
    client = client_for(factory, map_data={'width': 1})
    reply = client.post('/api/robots/pinky1/park', json={'slot': 'slot_10'})
    assert reply.status_code == 200 and reply.json()['task']['action'] == 'park'
    assert client.post('/api/robots/pinky1/goto', json={'x': 0, 'y': 0}).status_code == 409
    assert client.post('/api/robots/pinky1/park', json={'slot': 'nope'}).status_code == 404
    assert client.post('/api/robots/pinky2/goto', json={'x': 'a'}).status_code == 422
    assert client.post('/api/robots/pinky2/drive', json={'track': 1}).status_code == 400
    assert client.post('/api/robots/pinky2/gap').status_code == 501
    assert client.post('/api/stop').json() == {'canceled': {'pinky1': True, 'pinky2': False}}
    assert client.get('/api/log').json()[0]['state'] == CANCELED
    assert client.get('/api/map').json() == {'width': 1}
    assert client.get('/api/scenario').status_code == 404
    with client.websocket_connect('/api/ws') as socket:
        assert [r['name'] for r in socket.receive_json()['robots']] == ['pinky1', 'pinky2']


def test_http_arena_and_scenario(arena):
    data = read_yaml('scenario.yaml')
    runner = ScenarioRunner(arena, data, poll_s=0.01)
    client = client_for(arena, scenario=runner)
    assert client.get('/api/layout').json()['kind'] == 'arena'
    assert client.put('/api/robots/pinky1/vehicle', json={'type': 'T10'}).json()['type'] == 'T10'
    assert client.post('/api/robots/pinky2/fuel', json={'step': 'wait'}).status_code == 200
    assert client.post('/api/robots/pinky1/teleop', json={'linear': 1}).json()['linear'] == 0.1
    assert len(client.get('/api/scenario').json()['steps']) == len(data['steps'])
    assert client.post('/api/scenario/stop').status_code == 200


def test_team_password(factory):
    from pinky_parking_control.server import password_matches
    assert password_matches('', None) and password_matches('abc', 'abc')
    assert not password_matches('abc', 'abd') and not password_matches('abc', None)

    client = client_for(factory, password='team-secret')
    assert client.get('/api/auth').json() == {'required': True}
    assert client.get('/api/robots').status_code == 200   # 보기는 열려 있다
    assert client.post('/api/robots/pinky1/park', json={'slot': 'slot_10'}).status_code == 401
    wrong = {'X-Control-Password': 'wrong'}
    assert client.post('/api/auth/check', headers=wrong).status_code == 401
    ok = {'X-Control-Password': 'team-secret'}
    assert client.post('/api/auth/check', headers=ok).json() == {'ok': True}
    reply = client.post('/api/robots/pinky1/park', json={'slot': 'slot_10'}, headers=ok)
    assert reply.status_code == 200
    assert client.post('/api/stop').status_code == 200   # 멈추기는 언제나 된다


def test_park_poses_enter_from_open_end_and_skip_when_parked():
    layout = ArenaLayout(read_yaml('arena.yaml'))
    s10, s11 = layout.slots['slot_10'], layout.slots['slot_11']
    # 두 칸이 위아래로 붙어 있다: 10번은 아래에서 위로, 11번은 위에서 아래로 들어간다 (서로를 지나지 않음)
    low = layout.park_poses(s10)
    assert low[0]['y'] == pytest.approx(-0.43)
    assert low[0]['yaw'] == pytest.approx(math.pi / 2, abs=1e-3)
    high = layout.park_poses(s11)
    assert high[0]['y'] == pytest.approx(0.43)
    assert high[-1]['yaw'] == pytest.approx(-math.pi / 2, abs=1e-3)
    for entry in (low[0], high[0]):           # 진입 자리는 다른 칸 밖에 있다
        assert min(abs(entry['y'] - s10['y']), abs(entry['y'] - s11['y'])) >= 0.28
    up = {'x': 0.02, 'y': 0.16, 'yaw': math.pi / 2 + 0.05}            # 이미 칸 안에 바로 서 있다
    assert layout.park_poses(s11, up) == [{**s11, 'yaw': round(s11['yaw'], 4)}]
    down = {'x': 0.02, 'y': 0.16, 'yaw': -math.pi / 2 + 0.05}         # 반대로 서 있어도 주차된 것
    assert len(layout.park_poses(s11, down)) == 1
    crooked = {'x': 0.02, 'y': 0.16, 'yaw': math.pi / 2 + 0.4}        # 자리는 맞지만 비뚤다
    assert len(layout.park_poses(s11, crooked)) == 2


def test_precise_tolerance_for_parking_only():
    """주차·주유 진입은 좁은 도착 허용 오차, 그 밖은 화면에서 정한 값."""
    sent = []

    class TuneLink(FakeLink):
        def set_nav2_parameters(self, by_node):
            sent.append(by_node['controller_server'])
            return list(by_node)

    fleet = Fleet(ROBOTS, ArenaLayout(read_yaml('arena.yaml')), CONFIG, TuneLink, CALLSIGNS)
    try:
        fleet.park('pinky1', 'slot_10')
        assert sent[-1]['general_goal_checker.xy_goal_tolerance'] == 0.03
        fleet.cancel('pinky1')
        fleet.unpark('pinky1', 'slot_10')
        assert sent[-1]['general_goal_checker.xy_goal_tolerance'] == 0.03
        fleet.cancel('pinky1')
        fleet.drive('pinky1', 1)
        assert sent[-1]['general_goal_checker.xy_goal_tolerance'] == 0.08
        assert sent[-1]['general_goal_checker.yaw_goal_tolerance'] == 0.25
        fleet.cancel('pinky1')
        fleet.fuel('pinky1', 'enter')
        assert sent[-1]['general_goal_checker.xy_goal_tolerance'] == 0.03
    finally:
        fleet.close()


# ---- 주행 설정, SLAM 지도, 라이다 점 ------------------------------------------
def test_tuning_clean_and_nav2_parameters():
    from pinky_parking_control import tuning
    assert tuning.clean({'speed': 9, 'lookahead': '0.2'}) == {'speed': 0.25, 'lookahead': 0.2}
    for bad in ({'warp': 1}, {'speed': 'fast'}, {'speed': float('nan')}):
        with pytest.raises(ValueError):
            tuning.clean(bad)
    params = tuning.nav2_parameters({'speed': 0.1, 'turn': 0.8, 'lookahead': 0.2})
    assert params['controller_server']['FollowPath.desired_linear_vel'] == 0.1
    assert params['controller_server']['FollowPath.min_lookahead_dist'] == 0.2
    assert params['velocity_smoother']['max_velocity'] == [0.125, 0.0, 0.8]
    assert tuning.nav2_parameters({'goal_tolerance': 0.05}) == {
        'controller_server': {'general_goal_checker.xy_goal_tolerance': 0.05}}
    assert {t['key'] for t in tuning.describe({})} == set(tuning.SETTINGS)


def test_fleet_tuning_changes_teleop_limit_and_lane_status(arena):
    reply = arena.set_tuning({'teleop_linear': 0.05, 'speed': 0.12})
    values = {t['key']: t['value'] for t in reply['values']}
    assert values['teleop_linear'] == 0.05 and values['speed'] == 0.12
    assert arena.teleop('pinky1', 1.0, 0.0)['linear'] == 0.05      # 감도를 낮추면 직접 운전도 느려진다
    lane = arena.robot('pinky1')['lane']                            # FakeLink 는 1번 트랙 위에 있다
    assert lane['track'] == 1 and lane['offset'] < 0.005


def test_scan_points_and_grid_encoding():
    from types import SimpleNamespace
    from pinky_parking_control.mapdata import encode_grid
    from pinky_parking_control.mapdata import scan_points
    # 로봇이 원점에서 +x 를 본다. 라이다는 180° 돌아 있어 각도 0 의 빛은 로봇 뒤(-x)로 나간다
    pts = scan_points([1.0, float('inf'), 0.01], 0.0, math.pi, 8.0, {'x': 0, 'y': 0, 'yaw': 0})
    assert len(pts) == 1 and pts[0][0] == pytest.approx(-1.017, abs=1e-3)
    info = SimpleNamespace(width=2, height=2, resolution=0.05,
                           origin=SimpleNamespace(position=SimpleNamespace(x=-1.0, y=-2.0)))
    grid = encode_grid(SimpleNamespace(info=info, data=[0, 100, -1, 0]))   # 아랫줄부터 온다
    assert grid['cells'] == 'uf' + 'fo' and grid['origin'] == {'x': -1.0, 'y': -2.0}


# ---- 기준 경로 (가제보 등에서 기록한 주행 좌표) ---------------------------------
def write_reference(path):
    """시험용 기록: 1 m 직진 → 2초 정지 → 0.5 m 위로."""
    lines = ['t_s,step_id,step_name,sub_state,x_mm,y_mm,yaw_rad,yaw_deg']
    t = 0.0
    for i in range(101):                                   # x 0 → 1000 mm
        lines.append(f'{t:.2f},1,[1] 직진,,{i * 10:.1f},0.0,0.0,0.0')
        t += 0.05
    for _ in range(40):                                    # 정지
        lines.append(f'{t:.2f},1,[1] 직진,,1000.0,0.0,1.5708,90.0')
        t += 0.05
    for i in range(1, 51):                                 # y 0 → 500 mm
        lines.append(f'{t:.2f},2,[2] 위로,,1000.0,{i * 10:.1f},1.5708,90.0')
        t += 0.05
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return path


def test_reference_splits_at_stops_and_resamples(tmp_path):
    from pinky_parking_control import reference
    segments = reference.load_reference(write_reference(tmp_path / 'ref.csv'))
    assert [s['length'] for s in segments] == [1.0, 0.5]           # 멈춘 곳에서 둘로 나뉜다
    assert segments[0]['steps'] == ['[1] 직진'] and '[2] 위로' in segments[1]['steps']
    first = segments[0]['poses']
    assert first[0] == {'x': 0.0, 'y': 0.0, 'yaw': 0.0} and first[-1]['x'] == 1.0
    gaps = [math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(first, first[1:])]
    assert max(gaps) <= 0.045 and len(first) < 40                  # 3 cm 간격으로 다시 뽑는다
    # 로봇이 시작점에서 떨어져 있으면 그 사이를 잇는다
    lead = reference.with_approach(first, {'x': 0.0, 'y': -0.3, 'yaw': 0.0})
    assert lead[0]['y'] == -0.3 and len(lead) > len(first)


def test_reference_deviation():
    from pinky_parking_control import reference
    planned = [{'x': 0.0, 'y': 0.0}, {'x': 1.0, 'y': 0.0}]
    actual = [[0.0, 0.0, 0.02], [1.0, 0.5, 0.04], [2.0, 0.9, 0.0]]
    assert reference.deviation(planned, actual) == {'mean': 0.02, 'max': 0.04, 'end': 0.1}
    assert reference.deviation(planned, []) is None


def test_fleet_follows_reference_segment(tmp_path):
    config = dict(CONFIG, references={'pinky1': str(write_reference(tmp_path / 'ref.csv'))})
    fleet = Fleet(ROBOTS, ArenaLayout(read_yaml('arena.yaml')), config, FakeLink, CALLSIGNS)
    try:
        view = fleet.reference_view('pinky1')
        assert [s['length'] for s in view] == [1.0, 0.5] and fleet.reference_view('pinky2') == []
        task = fleet.follow_reference('pinky1', 1)
        assert task.action == 'reference' and task.mode == 'path'
        assert task.poses[-1] == {'x': 1.0, 'y': 0.5, 'yaw': 1.5708}
        assert len(fleet.layout_dict()['references']['pinky1']) == 2
        fleet.cancel('pinky1')
        for bad in (5, 'x', -1):
            with pytest.raises(UnknownName):
                fleet.follow_reference('pinky1', bad)
        with pytest.raises(UnknownName):
            fleet.follow_reference('pinky2', 0)
    finally:
        fleet.close()


def test_real_robot_pose_is_unconfirmed_until_set():
    """실물(배터리 값이 있다)은 켠 직후 위치가 가정일 뿐이다. 위치 맞추기나 마커 보정 뒤에 확인된다."""
    class RealLink(FakeLink):
        marker = None

        def status(self):
            return {**super().status(), 'battery_v': 7.4, 'marker': self.marker}

        def set_pose(self, pose):
            self.pose = dict(pose)

    fleet = Fleet(ROBOTS, ArenaLayout(read_yaml('arena.yaml')), CONFIG, RealLink, CALLSIGNS)
    try:
        assert fleet.robot('pinky1')['pose_confirmed'] is False
        fleet.set_pose('pinky1', slot='slot_11')
        assert fleet.robot('pinky1')['pose_confirmed'] is True
        assert fleet.robot('pinky2')['pose_confirmed'] is False
        fleet.links['pinky2'].marker = {'visible': [], 'last': {'id': 0, 'distance': 0.03}}
        assert fleet.robot('pinky2')['pose_confirmed'] is True         # 마커로 한 번 보정됐다
    finally:
        fleet.close()


def test_sim_robot_pose_is_confirmed_from_start(arena):
    assert arena.robot('pinky1')['pose_confirmed'] is True


# ---- 상태 알림: 왜 안 움직이는지 ----------------------------------------------
def test_health_reasons_and_failure_detail():
    from pinky_parking_control import health
    line = 'RegulatedPurePursuitController detected collision ahead!'
    assert '장애물' in health.reason_from_log(line)
    assert health.reason_from_log('Activating controller_server') == ''
    assert health.failure_detail(105, '') == (
        'Nav2 가 작업을 중단했습니다: 제자리에서 나아가지 못했습니다 (끼임, 바퀴 헛돎)')
    assert health.failure_detail(0, '앞에 장애물') == 'Nav2 가 작업을 중단했습니다: 앞에 장애물'
    assert health.failure_detail(0, '') == 'Nav2 가 작업을 중단했습니다'


def test_stall_watch_counts_only_while_not_moving():
    from pinky_parking_control.health import StallWatch
    watch = StallWatch()
    here = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    assert watch.update(None, here, 0.0) == 0.0                 # 작업이 없으면 세지 않는다
    assert watch.update(7, here, 10.0) == 0.0                   # 작업 시작
    assert watch.update(7, here, 15.0) == 5.0                   # 5초째 그대로
    assert watch.update(7, {'x': 0.05, 'y': 0.0, 'yaw': 0.0}, 16.0) == 0.0     # 움직였다
    assert watch.update(7, {'x': 0.05, 'y': 0.0, 'yaw': 0.2}, 17.0) == 0.0     # 제자리 회전도 움직임
    assert watch.update(8, {'x': 0.05, 'y': 0.0, 'yaw': 0.2}, 30.0) == 0.0     # 새 작업은 다시 센다


def test_alerts_order_and_texts():
    from pinky_parking_control import health
    base = {'odom_hz': 20.0, 'nav2_ready': True, 'localized': True, 'battery_v': 7.8,
            'pose_confirmed': True, 'task': None, 'issue': None}
    assert health.alerts(base) == []
    low = health.alerts({**base, 'battery_v': 6.9})
    assert low[0]['level'] == 'error' and '충전' in low[0]['text']
    assert health.alerts({**base, 'battery_v': 7.2})[0]['level'] == 'warning'
    gone = health.alerts({**base, 'odom_hz': 0})
    assert gone[0] == {'level': 'error', 'text': '로봇 응답 없음: 전원과 와이파이를 확인하세요'}
    running = {'state': 'running', 'target': '1번 트랙'}
    issue = {'text': '앞에 장애물이 있어 멈췄습니다 (라이다가 본 물체)', 'age_s': 1.0}
    stuck = health.alerts({**base, 'task': running, 'issue': issue}, stalled_s=6.0)
    assert stuck[0]['level'] == 'error' and '6초째 움직이지 않습니다' in stuck[0]['text']
    assert '장애물' in stuck[0]['text']
    moving = health.alerts({**base, 'task': running, 'issue': issue}, stalled_s=1.0)
    assert moving == [{'level': 'warning', 'text': issue['text']}]     # 움직이는 중이면 경고만
    old = health.alerts({**base, 'task': running, 'issue': {**issue, 'age_s': 30}}, stalled_s=6.0)
    assert '사유를 알 수 없습니다' in old[0]['text']                    # 오래된 경고는 사유로 안 쓴다
    failed = {'state': 'failed', 'target': '2번 트랙', 'detail': '경로 없음', 'finished': 100.0}
    assert '2번 트랙 실패: 경로 없음' in health.alerts({**base, 'task': failed}, now=150.0)[0]['text']
    assert health.alerts({**base, 'task': failed}, now=500.0) == []    # 2분 지나면 내린다


def test_fleet_status_has_alerts_and_stall(arena):
    arena.drive('pinky1', 1)
    for _ in range(3):                      # 감시 루프가 한 번 돌 때까지
        time.sleep(0.15)
    status = arena.robot('pinky1')
    assert 'alerts' in status and status['stalled_s'] >= 0.0
