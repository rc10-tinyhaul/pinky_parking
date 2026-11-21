"""
관제 서버: 행동 단위 HTTP API, 실시간 상태 WebSocket, 관제센터 대시보드.

  ros2 run pinky_parking_control control_server
  ros2 run pinky_parking_control control_server --layout arena --port 8080

API (자세한 형식은 http://<host>:8080/docs):
  기본 이동   POST /api/robots/{name}/goto {x, y, yaw}  /teleop {linear, angular}
              /cancel  /stop   POST /api/stop
  주차·출차   POST /api/robots/{name}/park {slot}  /unpark {slot}  /park/precise (준비 중)
  주행        POST /api/robots/{name}/drive {track}  /lane_change {to}  /lap {route}
              /gap (준비 중)
  주유        POST /api/robots/{name}/fuel {step: wait | enter | leave}
  차종        PUT  /api/robots/{name}/vehicle {type: T00 | T10 | T15}
  시나리오    GET /api/scenario   POST /api/scenario/{next | run | stop | confirm | reset}
  상태 조회   GET /api/robots  /api/robots/{name}  /api/log  /api/layout  /api/map  WS /api/ws
  팀 비밀번호 GET /api/auth   POST /api/auth/check

환경변수 PINKY_CONTROL_PASSWORD 를 정하면 움직이는 명령에 X-Control-Password 머리글이 필요하다.
멈추는 명령(cancel, stop, 시나리오 stop)은 언제나 받는다.
"""
import argparse
import asyncio
import hmac
import os
from pathlib import Path
import sys
import time

from fastapi import APIRouter
from fastapi import Body
from fastapi import Depends
from fastapi import FastAPI
from fastapi import Header
from fastapi import HTTPException
from fastapi import WebSocket
from fastapi import WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pinky_parking_control.fleet import UnknownName
from pinky_parking_control.layout import NotInLayout
from pinky_parking_control.model import NotReady
from pinky_parking_control.model import NotYet
from pinky_parking_control.model import TaskBusy
import yaml

PASSWORD_ENV = 'PINKY_CONTROL_PASSWORD'


def _call(fn, *args):
    """도메인 오류를 HTTP 상태로 바꾼다."""
    try:
        return fn(*args)
    except UnknownName as error:
        raise HTTPException(404, str(error))
    except TaskBusy as error:
        raise HTTPException(409, str(error))
    except NotInLayout as error:
        raise HTTPException(400, str(error))
    except NotYet as error:
        raise HTTPException(501, str(error))
    except NotReady as error:
        raise HTTPException(503, str(error))
    except (ValueError, KeyError, TypeError) as error:
        raise HTTPException(422, str(error))
    except RuntimeError as error:
        raise HTTPException(409, str(error))


def password_matches(expected, given):
    """팀 비밀번호 비교. 비밀번호를 정하지 않았으면(빈 값) 누구나 통과한다."""
    if not expected:
        return True
    return hmac.compare_digest(str(given or '').encode(), str(expected).encode())


def create_app(fleet, map_data=None, static_dir=None, status_rate_hz=10.0, password='',
               scenario=None):
    """
    관제 API 를 만든다.

    password 를 주면 로봇을 움직이는 명령은 X-Control-Password 머리글이 맞아야 받는다.
    상태 보기(GET, WebSocket)와 멈추는 명령(cancel, stop)은 비밀번호 없이 받는다.
    """
    app = FastAPI(title='Pinky 관제 API', version='0.2.0',
                  description='로봇마다 행동 단위(이동, 주차·출차, 트랙 주행, 차선 변경, 주유, 차종, '
                              '직접 운전)로 명령하고, 시연 시나리오를 단계별로 실행합니다. '
                              '팀 비밀번호를 정했으면 명령마다 X-Control-Password 머리글에 넣습니다.')
    period = 1.0 / max(1.0, float(status_rate_hz))

    def guard(x_control_password: str = Header(default='')):
        if not password_matches(password, x_control_password):
            time.sleep(1.0)   # 비밀번호를 마구 넣어 보는 것을 늦춘다
            raise HTTPException(401, '팀 비밀번호가 틀렸거나 없습니다')

    commands = APIRouter(dependencies=[Depends(guard)])

    def task_reply(task):
        return {'accepted': True, 'task': task.to_dict()}

    def scenario_or_404():
        if scenario is None:
            raise HTTPException(404, '시나리오는 경기장 배치에서만 있습니다')
        return scenario

    def snapshot():
        snap = fleet.snapshot()
        if scenario is not None:
            snap['scenario'] = scenario.status()
        return snap

    # ---- 팀 비밀번호 ------------------------------------------------------
    @app.get('/api/auth', tags=['팀 비밀번호'])
    def auth_required():
        return {'required': bool(password)}

    @commands.post('/api/auth/check', tags=['팀 비밀번호'])
    def auth_check():
        return {'ok': True}

    # ---- 상태 조회 -------------------------------------------------------
    @app.get('/api/robots', tags=['상태 조회'])
    def robots():
        return fleet.snapshot()['robots']

    @app.get('/api/robots/{name}', tags=['상태 조회'])
    def robot(name: str):
        return _call(fleet.robot, name)

    @app.get('/api/log', tags=['상태 조회'])
    def log(limit: int = 50):
        return fleet.log.recent(limit)

    @app.get('/api/layout', tags=['상태 조회'])
    def layout():
        return fleet.layout_dict()

    @app.get('/api/map', tags=['상태 조회'])
    def map_():
        if map_data is None:
            raise HTTPException(404, '지도가 없습니다 (경기장 배치는 도면으로 그립니다)')
        return map_data

    @app.get('/api/robots/{name}/slam_map', tags=['상태 조회'])
    def slam_map(name: str):
        """SLAM 이 만들고 있는 지도 (localization:=slam 일 때). 없으면 404."""
        link = _call(fleet._link, name)
        view = link.slam_view() if hasattr(link, 'slam_view') else None
        if view is None:
            raise HTTPException(404, 'SLAM 지도가 없습니다')
        return view

    @app.get('/api/robots/{name}/scan', tags=['상태 조회'])
    def scan(name: str):
        """지금 라이다가 보는 점들 (지도 좌표)."""
        link = _call(fleet._link, name)
        return link.scan_view() if hasattr(link, 'scan_view') else {'points': []}

    @app.get('/api/tuning', tags=['주행 설정'])
    def tuning_get():
        return fleet.tuning_view()

    @commands.post('/api/tuning', tags=['주행 설정'])
    def tuning_set(body: dict = Body(..., examples=[{'speed': 0.15, 'lookahead': 0.2}])):
        """속도·회전·곡선 감도·도착 허용 거리·직접 운전 감도를 바꾼다. 달리는 중에도 적용된다."""
        return _call(fleet.set_tuning, body)

    @commands.post('/api/robots/{name}/set_pose', tags=['기본 이동'])
    def set_pose(name: str, body: dict = Body(..., examples=[{'slot': 'slot_11'}])):
        """로봇을 손으로 옮겨 놓은 뒤 지금 위치를 알려 준다: {slot} 또는 {x, y, yaw}."""
        if body.get('slot'):
            return _call(fleet.set_pose, name, None, str(body['slot']))
        return _call(fleet.set_pose, name, body)

    @commands.post('/api/robots/{name}/markers/register', tags=['기본 이동'])
    def markers_register(name: str, body: dict = Body(..., examples=[{'slot': 'slot_11'}])):
        """지금 카메라에 보이는 마커의 위치를 등록한다: {slot} 이면 그 칸 기준, 비우면 지금 위치 기준."""
        return _call(fleet.register_markers, name, str(body.get('slot') or ''))

    @app.websocket('/api/ws')
    async def stream(socket: WebSocket):
        await socket.accept()
        try:
            while True:
                await socket.send_json(snapshot())
                await asyncio.sleep(period)
        except (WebSocketDisconnect, RuntimeError):
            pass

    # ---- 기본 이동 -------------------------------------------------------
    @commands.post('/api/robots/{name}/goto', tags=['기본 이동'])
    def goto(name: str, pose: dict = Body(..., examples=[{'x': 0.3, 'y': -1.1, 'yaw': 0.0}])):
        return task_reply(_call(fleet.goto, name, pose))

    @commands.post('/api/robots/{name}/teleop', tags=['기본 이동'])
    def teleop(name: str, body: dict = Body(..., examples=[{'linear': 0.1, 'angular': 0.0}])):
        """직접 운전. 0.5 초 안에 다시 보내지 않으면 멈춘다 (손을 떼면 선다)."""
        return _call(fleet.teleop, name, body.get('linear', 0.0), body.get('angular', 0.0))

    # 멈추는 명령(취소, 정지)은 안전을 위해 비밀번호 없이도 받는다
    @app.post('/api/robots/{name}/cancel', tags=['기본 이동'])
    def cancel(name: str):
        return {'canceled': _call(fleet.cancel, name)}

    @app.post('/api/robots/{name}/stop', tags=['기본 이동'])
    def stop(name: str):
        return {'canceled': _call(fleet.stop, name)}

    @app.post('/api/stop', tags=['기본 이동'])
    def stop_all():
        if scenario is not None:
            scenario.stop()
        return {'canceled': fleet.stop_all()}

    # ---- 주차·출차 -------------------------------------------------------
    @commands.post('/api/robots/{name}/park', tags=['주차·출차'])
    def park(name: str, body: dict = Body(..., examples=[{'slot': 'slot_10'}])):
        return task_reply(_call(fleet.park, name, str(body.get('slot', ''))))

    @commands.post('/api/robots/{name}/unpark', tags=['주차·출차'])
    def unpark(name: str, body: dict = Body(..., examples=[{'slot': 'slot_10'}])):
        return task_reply(_call(fleet.unpark, name, str(body.get('slot', ''))))

    @commands.post('/api/robots/{name}/park/precise', tags=['주차·출차'])
    def park_precise(name: str):
        """주차 칸 정밀 정렬 (준비 중: #7, #8)."""
        return _call(fleet.pending, 'park_precise')

    # ---- 주행 ------------------------------------------------------------
    @commands.post('/api/robots/{name}/drive', tags=['주행'])
    def drive(name: str, body: dict = Body(..., examples=[{'track': 1, 'laps': 1}])):
        return task_reply(_call(fleet.drive, name, body.get('track'), body.get('laps', 1)))

    @commands.post('/api/robots/{name}/lane_change', tags=['주행'])
    def lane_change(name: str, body: dict = Body(..., examples=[{'to': 2}])):
        return task_reply(_call(fleet.lane_change, name, body.get('to')))

    @commands.post('/api/robots/{name}/lap', tags=['주행'])
    def lap(name: str, body: dict = Body(..., examples=[{'route': 'lap'}])):
        return task_reply(_call(fleet.lap, name, str(body.get('route', 'lap'))))

    @app.get('/api/robots/{name}/reference', tags=['주행'])
    def reference_list(name: str):
        """기준 경로(가제보 등에서 기록한 주행 좌표)의 구간 목록."""
        return _call(fleet.reference_view, name)

    @commands.post('/api/robots/{name}/reference', tags=['주행'])
    def reference_follow(name: str, body: dict = Body(..., examples=[{'segment': 0}])):
        """기준 경로의 구간 하나를 따라간다. 로봇이 구간 시작점 근처에 있어야 한다."""
        return task_reply(_call(fleet.follow_reference, name, body.get('segment')))

    @commands.post('/api/robots/{name}/gap', tags=['주행'])
    def gap(name: str):
        """앞뒤 간격 조절 (준비 중: #12)."""
        return _call(fleet.pending, 'gap')

    # ---- 주유 ------------------------------------------------------------
    @commands.post('/api/robots/{name}/fuel', tags=['주유'])
    def fuel(name: str, body: dict = Body(..., examples=[{'step': 'wait'}])):
        return task_reply(_call(fleet.fuel, name, str(body.get('step', ''))))

    # ---- 차종 ------------------------------------------------------------
    @commands.put('/api/robots/{name}/vehicle', tags=['차종'])
    def vehicle(name: str, body: dict = Body(..., examples=[{'type': 'T10'}])):
        return _call(fleet.set_vehicle, name, str(body.get('type', '')))

    # ---- 시연 시나리오 ---------------------------------------------------
    @app.get('/api/scenario', tags=['시연 시나리오'])
    def scenario_status():
        return scenario_or_404().status()

    @commands.post('/api/scenario/next', tags=['시연 시나리오'])
    def scenario_next(body: dict = Body(default={})):
        """다음 단계 하나 (index 를 주면 그 단계)."""
        return _call(scenario_or_404().start, body.get('index'), False)

    @commands.post('/api/scenario/run', tags=['시연 시나리오'])
    def scenario_run(body: dict = Body(default={})):
        """다음 단계부터 끝까지 이어서."""
        return _call(scenario_or_404().start, body.get('index'), True)

    @commands.post('/api/scenario/confirm', tags=['시연 시나리오'])
    def scenario_confirm():
        return _call(scenario_or_404().confirm)

    @commands.post('/api/scenario/reset', tags=['시연 시나리오'])
    def scenario_reset():
        return _call(scenario_or_404().reset)

    @app.post('/api/scenario/stop', tags=['시연 시나리오'])
    def scenario_stop():
        return scenario_or_404().stop()

    app.include_router(commands)
    if static_dir is not None and Path(static_dir).is_dir():
        # colcon --symlink-install 은 static 파일을 build/ 로 가는 링크로 설치한다. 링크를 따라가야 화면이 열린다.
        app.mount('/', StaticFiles(directory=str(static_dir), html=True, follow_symlink=True),
                  name='dashboard')
    return app


def _share(package):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))


def _yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}


def main():
    import rclpy
    from pinky_parking_bringup.robots import load_robots
    from pinky_parking_control.fleet import Fleet
    from pinky_parking_control.layout import ARENA
    from pinky_parking_control.layout import ArenaLayout
    from pinky_parking_control.layout import choose_layout
    from pinky_parking_control.layout import FactoryLayout
    from pinky_parking_control.mapdata import load_map
    from pinky_parking_control.robot_link import RobotLink
    from pinky_parking_control.scenario import ScenarioRunner
    import uvicorn

    share = _share('pinky_parking_control')
    parser = argparse.ArgumentParser(description='관제 서버 (API + 대시보드)')
    parser.add_argument('--config', default=str(share / 'config/control.yaml'))
    parser.add_argument('--arena', default=str(share / 'config/arena.yaml'))
    parser.add_argument('--scenario', default=str(share / 'config/scenario.yaml'))
    parser.add_argument('--robots', default=str(_share('pinky_parking_bringup')
                                                / 'config/robots.yaml'))
    parser.add_argument('--slots', default=str(_share('pinky_parking_docking')
                                               / 'config/slots.yaml'))
    parser.add_argument('--routes', default=str(_share('pinky_parking_driving')
                                                / 'config/route.yaml'))
    parser.add_argument('--map', default=str(_share('pinky_parking_navigation')
                                             / 'maps/factory.yaml'))
    parser.add_argument('--layout', choices=['auto', 'arena', 'factory'])
    parser.add_argument('--room-map', default='',
                        help='경기장 배치에서 겹쳐 보여 줄 고정벽 지도 YAML (snapshot_map 이 만든 것)')
    parser.add_argument('--log-dir', default=os.environ.get('PINKY_LOG_DIR', ''),
                        help='작업마다 궤적 파일을 남길 폴더 (기본: 환경변수 PINKY_LOG_DIR, 없으면 ~/pinky_logs)')
    parser.add_argument('--camera', action='append', default=[], metavar='이름=주소',
                        help='로봇 카메라 영상 주소 (예: pinky1=http://192.168.0.5:8090/stream). 여러 번')
    parser.add_argument('--reference', action='append', default=[], metavar='이름=CSV',
                        help='기준 경로 CSV (예: pinky1=/data/coords_robot_a.csv). 여러 번')
    parser.add_argument('--only', default='',
                        help='일부 로봇만 연결: 이름을 쉼표로 (예: pinky1). 실물을 한 대만 시험할 때')
    parser.add_argument('--host')
    parser.add_argument('--port', type=int)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    config = _yaml(args.config)
    log_dir = Path(args.log_dir).expanduser() if args.log_dir else Path.home() / 'pinky_logs'
    config['trajectory_dir'] = str(log_dir / 'trajectories')
    cameras = dict(config.get('cameras') or {})
    cameras.update(item.split('=', 1) for item in args.camera if '=' in item)
    config['cameras'] = cameras
    references = dict(config.get('references') or {})
    references.update(item.split('=', 1) for item in args.reference if '=' in item)
    config['references'] = references
    world_name, robots = load_robots(args.robots)
    only = [n for n in args.only.split(',') if n]
    if only:
        robots = [r for r in robots if r['name'] in only]
        if not robots:
            parser.error(f'--only 에 맞는 로봇이 없습니다: {args.only}')
    arena = _yaml(args.arena)
    kind = choose_layout(args.layout or config.get('layout', 'auto'), world_name)
    if kind == ARENA:
        room = Path(args.room_map).expanduser() if args.room_map else None
        layout = ArenaLayout(arena)
        map_data = load_map(room) if room and room.is_file() else None
        scenario_data = _yaml(args.scenario)
    else:
        layout = FactoryLayout(_yaml(args.slots)['slots'], _yaml(args.routes)['routes'],
                               config.get('unpark_exits'), arena['vehicles'])
        map_data = load_map(args.map) if Path(args.map).is_file() else None
        scenario_data = None

    callsigns = (scenario_data or _yaml(args.scenario)).get('callsigns')
    # 팀 비밀번호는 저장소에 남지 않게 환경변수로만 받는다 (compose.yaml 이 호스트 값을 넘긴다)
    password = os.environ.get(PASSWORD_ENV, '')
    fleet = Fleet(robots, layout, config, RobotLink, callsigns)
    scenario = ScenarioRunner(fleet, scenario_data) if scenario_data else None
    app = create_app(fleet, map_data, share / 'static', config.get('status_rate_hz', 10),
                     password, scenario)
    host, port = args.host or config.get('host', '0.0.0.0'), args.port or config.get('port', 8080)
    names = ', '.join(f'{r["name"]}(도메인 {r["domain_id"]})' for r in robots)
    print(f'관제 서버: http://{host}:{port}  배치: {layout.to_dict()["name"]}  로봇: {names}',
          flush=True)
    print(f'궤적 기록: {config["trajectory_dir"]}', flush=True)
    if password:
        print('팀 비밀번호: 켜짐 (명령을 보내려면 비밀번호가 필요합니다)', flush=True)
    else:
        print(f'팀 비밀번호: 꺼짐. 다른 PC 에서 접속하게 하려면 {PASSWORD_ENV} 를 정하세요 '
              '(docs/remote_access.md)', flush=True)
    try:
        uvicorn.run(app, host=host, port=int(port), log_level='warning')
    finally:
        if scenario is not None:
            scenario.stop()
        fleet.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
