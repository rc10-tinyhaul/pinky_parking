"""관제 API 의 순수 로직 시험 (ROS 없이)."""
import math
from pathlib import Path

from pinky_parking_control.fleet import Fleet
from pinky_parking_control.fleet import UnknownName
from pinky_parking_control.mapdata import classify
from pinky_parking_control.mapdata import load_map
from pinky_parking_control.mapdata import read_pgm
from pinky_parking_control.model import CANCELED
from pinky_parking_control.model import parse_pose
from pinky_parking_control.model import quaternion_to_yaw
from pinky_parking_control.model import RUNNING
from pinky_parking_control.model import SUCCEEDED
from pinky_parking_control.model import Task
from pinky_parking_control.model import TaskBusy
from pinky_parking_control.model import TaskLog
from pinky_parking_control.model import yaw_to_quaternion
import pytest

ROBOTS = [{'name': 'pinky1', 'domain_id': 11}, {'name': 'pinky2', 'domain_id': 12}]
SLOTS = {'slot_10': {'x': 0.3, 'y': -1.1, 'yaw': 0.0}}
ROUTES = {'lap': [{'x': -0.8, 'y': 1.15, 'yaw': 3.14159}, {'x': 0.5, 'y': 1.0, 'yaw': 0.0}]}
CONFIG = {'timeout_s': {'goto': 5}, 'unpark_exits': {'slot_10': {'x': 0.0, 'y': 0.0}}}


class FakeLink:
    """RobotLink 대신: Nav2 없이 작업을 받아 두기만 한다."""

    def __init__(self, name, domain_id, on_finish):
        self.name, self.domain_id, self.on_finish = name, domain_id, on_finish
        self.task, self.stopped, self.closed = None, False, False

    def start(self, task):
        if self.task is not None and not self.task.done:
            raise TaskBusy('busy')
        self.task = task
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

    def status(self):
        return {'name': self.name, 'task': self.task.to_dict() if self.task else None}

    def close(self):
        self.closed = True


@pytest.fixture
def fleet():
    f = Fleet(ROBOTS, SLOTS, ROUTES, CONFIG, FakeLink)
    yield f
    f.close()


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


def test_fleet_actions(fleet):
    park = fleet.park('pinky1', 'slot_10')
    assert park.action == 'park' and park.poses == [{'x': 0.3, 'y': -1.1, 'yaw': 0.0}]
    assert park.timeout_s == 180.0
    with pytest.raises(TaskBusy):
        fleet.goto('pinky1', {'x': 0, 'y': 0})
    lap = fleet.lap('pinky2', 'lap')
    assert len(lap.poses) == 2 and lap.target == 'lap'

    assert fleet.cancel('pinky1') is True
    assert fleet.log.recent()[0]['state'] == CANCELED
    goto = fleet.goto('pinky1', {'x': 1, 'y': 2, 'yaw': 0})
    assert goto.target == '(1.00, 2.00)' and goto.timeout_s == 5.0

    stopped = fleet.stop_all()
    assert stopped == {'pinky1': True, 'pinky2': True}
    assert all(link.stopped for link in fleet.links.values())
    unpark = fleet.unpark('pinky1', 'slot_10')
    assert unpark.poses == [{'x': 0.0, 'y': 0.0, 'yaw': 0.0}]


def test_fleet_unknown_names(fleet):
    for call, args in ((fleet.park, ('pinky9', 'slot_10')), (fleet.park, ('pinky1', 'slot_99')),
                       (fleet.lap, ('pinky1', 'nowhere')), (fleet.unpark, ('pinky1', 'slot_11'))):
        with pytest.raises(UnknownName):
            call(*args)


def test_snapshot_shape(fleet):
    fleet.park('pinky1', 'slot_10')
    snap = fleet.snapshot()
    assert [r['name'] for r in snap['robots']] == ['pinky1', 'pinky2']
    assert snap['robots'][0]['task']['goal'] == {'x': 0.3, 'y': -1.1, 'yaw': 0.0}
    assert snap['robots'][1]['task'] is None


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


def test_http_api(fleet):
    pytest.importorskip('fastapi')
    pytest.importorskip('httpx')
    from fastapi.testclient import TestClient
    from pinky_parking_control.server import create_app

    client = TestClient(create_app(fleet, map_data={'width': 1}))
    reply = client.post('/api/robots/pinky1/park', json={'slot': 'slot_10'})
    assert reply.status_code == 200 and reply.json()['task']['action'] == 'park'
    assert client.post('/api/robots/pinky1/goto', json={'x': 0, 'y': 0}).status_code == 409
    assert client.post('/api/robots/pinky1/park', json={'slot': 'nope'}).status_code == 404
    assert client.post('/api/robots/pinky2/goto', json={'x': 'a'}).status_code == 422
    assert client.post('/api/stop').json() == {'canceled': {'pinky1': True, 'pinky2': False}}
    assert client.get('/api/log').json()[0]['state'] == CANCELED
    assert client.get('/api/map').json() == {'width': 1}
    with client.websocket_connect('/api/ws') as socket:
        assert [r['name'] for r in socket.receive_json()['robots']] == ['pinky1', 'pinky2']


def test_team_password(fleet):
    pytest.importorskip('fastapi')
    pytest.importorskip('httpx')
    from fastapi.testclient import TestClient
    from pinky_parking_control.server import create_app
    from pinky_parking_control.server import password_matches

    assert password_matches('', None) and password_matches('abc', 'abc')
    assert not password_matches('abc', 'abd') and not password_matches('abc', None)

    client = TestClient(create_app(fleet, password='team-secret'))
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
