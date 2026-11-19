"""
관제 서버: 행동 단위 HTTP API, 실시간 상태 WebSocket, 관제실 대시보드.

  ros2 run pinky_parking_control control_server
  ros2 run pinky_parking_control control_server --port 8080 --robots <robots.yaml>

API (자세한 형식은 http://<host>:8080/docs):
  기본 이동   POST /api/robots/{name}/goto {x, y, yaw}   /cancel   /stop    POST /api/stop
  주차·출차   POST /api/robots/{name}/park {slot}        /unpark {slot}
  주행        POST /api/robots/{name}/lap {route}
  상태 조회   GET /api/robots   GET /api/robots/{name}   GET /api/log   WS /api/ws
  배치        GET /api/layout (주차 칸, 경로, 출차 위치)  GET /api/map
  팀 비밀번호 GET /api/auth (필요한지)  POST /api/auth/check

환경변수 PINKY_CONTROL_PASSWORD 를 정하면 움직이는 명령(goto, park, unpark, lap)에
X-Control-Password 머리글이 필요하다. 멈추는 명령(cancel, stop)은 언제나 받는다.
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
from pinky_parking_control.model import NotReady
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
    except NotReady as error:
        raise HTTPException(503, str(error))
    except ValueError as error:
        raise HTTPException(422, str(error))


def password_matches(expected, given):
    """팀 비밀번호 비교. 비밀번호를 정하지 않았으면(빈 값) 누구나 통과한다."""
    if not expected:
        return True
    return hmac.compare_digest(str(given or '').encode(), str(expected).encode())


def create_app(fleet, map_data=None, static_dir=None, status_rate_hz=10.0, password=''):
    """
    관제 API 를 만든다.

    password 를 주면 로봇을 움직이는 명령은 X-Control-Password 머리글이 맞아야 받는다.

    상태 보기(GET, WebSocket)와 멈추는 명령(cancel, stop)은 비밀번호 없이 받는다.
    """
    app = FastAPI(title='Pinky 관제 API', version='0.1.0',
                  description='로봇마다 행동 단위(기본 이동, 주차·출차, 주행, 상태 조회)로 명령합니다. '
                              '팀 비밀번호를 정했으면 명령마다 X-Control-Password 머리글에 넣습니다.')
    period = 1.0 / max(1.0, float(status_rate_hz))

    def guard(x_control_password: str = Header(default='')):
        if not password_matches(password, x_control_password):
            time.sleep(1.0)   # 비밀번호를 마구 넣어 보는 것을 늦춘다
            raise HTTPException(401, '팀 비밀번호가 틀렸거나 없습니다')

    commands = APIRouter(dependencies=[Depends(guard)])

    def task_reply(task):
        return {'accepted': True, 'task': task.to_dict()}

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
        return fleet.layout()

    @app.get('/api/map', tags=['상태 조회'])
    def map_():
        if map_data is None:
            raise HTTPException(404, '지도가 없습니다')
        return map_data

    @app.websocket('/api/ws')
    async def stream(socket: WebSocket):
        await socket.accept()
        try:
            while True:
                await socket.send_json(fleet.snapshot())
                await asyncio.sleep(period)
        except (WebSocketDisconnect, RuntimeError):
            pass

    # ---- 기본 이동 -------------------------------------------------------
    @commands.post('/api/robots/{name}/goto', tags=['기본 이동'])
    def goto(name: str, pose: dict = Body(..., examples=[{'x': 0.3, 'y': -1.1, 'yaw': 0.0}])):
        return task_reply(_call(fleet.goto, name, pose))

    # 멈추는 명령(취소, 정지)은 안전을 위해 비밀번호 없이도 받는다
    @app.post('/api/robots/{name}/cancel', tags=['기본 이동'])
    def cancel(name: str):
        return {'canceled': _call(fleet.cancel, name)}

    @app.post('/api/robots/{name}/stop', tags=['기본 이동'])
    def stop(name: str):
        return {'canceled': _call(fleet.stop, name)}

    @app.post('/api/stop', tags=['기본 이동'])
    def stop_all():
        return {'canceled': fleet.stop_all()}

    # ---- 주차·출차 -------------------------------------------------------
    @commands.post('/api/robots/{name}/park', tags=['주차·출차'])
    def park(name: str, body: dict = Body(..., examples=[{'slot': 'slot_10'}])):
        return task_reply(_call(fleet.park, name, str(body.get('slot', ''))))

    @commands.post('/api/robots/{name}/unpark', tags=['주차·출차'])
    def unpark(name: str, body: dict = Body(..., examples=[{'slot': 'slot_10'}])):
        return task_reply(_call(fleet.unpark, name, str(body.get('slot', ''))))

    # ---- 주행 ------------------------------------------------------------
    @commands.post('/api/robots/{name}/lap', tags=['주행'])
    def lap(name: str, body: dict = Body(..., examples=[{'route': 'lap'}])):
        return task_reply(_call(fleet.lap, name, str(body.get('route', 'lap'))))

    app.include_router(commands)
    if static_dir is not None and Path(static_dir).is_dir():
        app.mount('/', StaticFiles(directory=str(static_dir), html=True), name='dashboard')
    return app


def _share(package):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))


def main():
    import rclpy
    from pinky_parking_bringup.robots import load_robots
    from pinky_parking_control.fleet import Fleet
    from pinky_parking_control.mapdata import load_map
    from pinky_parking_control.robot_link import RobotLink
    import uvicorn

    share = _share('pinky_parking_control')
    parser = argparse.ArgumentParser(description='관제 서버 (API + 대시보드)')
    parser.add_argument('--config', default=str(share / 'config/control.yaml'))
    parser.add_argument('--robots', default=str(_share('pinky_parking_bringup')
                                                / 'config/robots.yaml'))
    parser.add_argument('--slots', default=str(_share('pinky_parking_docking')
                                               / 'config/slots.yaml'))
    parser.add_argument('--routes', default=str(_share('pinky_parking_driving')
                                                / 'config/route.yaml'))
    parser.add_argument('--map', default=str(_share('pinky_parking_navigation')
                                             / 'maps/factory.yaml'))
    parser.add_argument('--host')
    parser.add_argument('--port', type=int)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    config = yaml.safe_load(Path(args.config).read_text()) or {}
    _, robots = load_robots(args.robots)
    slots = yaml.safe_load(Path(args.slots).read_text())['slots']
    routes = yaml.safe_load(Path(args.routes).read_text())['routes']
    map_data = load_map(args.map) if Path(args.map).is_file() else None

    # 팀 비밀번호는 저장소에 남지 않게 환경변수로만 받는다 (compose.yaml 이 호스트 값을 넘긴다)
    password = os.environ.get(PASSWORD_ENV, '')
    fleet = Fleet(robots, slots, routes, config, RobotLink)
    app = create_app(fleet, map_data, share / 'static', config.get('status_rate_hz', 10), password)
    host, port = args.host or config.get('host', '0.0.0.0'), args.port or config.get('port', 8080)
    names = ', '.join(f'{r["name"]}(도메인 {r["domain_id"]})' for r in robots)
    print(f'관제 서버: http://{host}:{port}  로봇: {names}', flush=True)
    if password:
        print('팀 비밀번호: 켜짐 (명령을 보내려면 비밀번호가 필요합니다)', flush=True)
    else:
        print(f'팀 비밀번호: 꺼짐. 다른 PC 에서 접속하게 하려면 {PASSWORD_ENV} 를 정하세요 '
              '(docs/remote_access.md)', flush=True)
    try:
        uvicorn.run(app, host=host, port=int(port), log_level='warning')
    finally:
        fleet.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
