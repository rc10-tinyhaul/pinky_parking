# pinky_parking_control

관제실 패키지입니다. 로봇에게 **행동 단위**(이동, 주차, 출차, 주행, 정지)로 명령하는 API와, 브라우저에서 로봇을 보고 움직이는 **대시보드**가 들어 있습니다.

## 한눈에 보기

```
 브라우저 (대시보드)                 관제 서버 (control_server, 프로세스 하나)
 ┌─────────────────┐   HTTP 명령    ┌──────────────────────────────────────┐
 │ 지도 · 로봇 카드 │ ────────────▶ │ FastAPI                              │
 │ 버튼 · 작업 기록 │ ◀──────────── │   ├─ 도메인 11 연결 ── pinky1 의 Nav2 │
 └─────────────────┘  WebSocket    │   └─ 도메인 12 연결 ── pinky2 의 Nav2 │
                      초당 10번 상태 └──────────────────────────────────────┘
```

| 용어 | 쉬운 뜻 |
|---|---|
| API | 프로그램끼리 주고받는 "주문서 양식". 대시보드 버튼도 결국 이 주문서를 보냅니다 |
| WebSocket | 서버가 화면에 계속 새 소식을 밀어 주는 "열린 전화선". 새로고침 없이 로봇이 움직여 보입니다 |
| 도메인(`ROS_DOMAIN_ID`) | 로봇마다 따로 쓰는 "무전 채널". pinky1 = 11번, pinky2 = 12번 |
| Nav2 | 로봇이 목표까지 길을 찾아 가게 해 주는 ROS 2 주행 프로그램 |

**왜 이렇게 만들었나요?** Nav2에 보내는 명령(액션)은 도메인을 건너가지 못합니다. 그래서 서버 하나가 로봇마다 그 로봇의 채널(도메인)에 직접 들어가 명령합니다. `domain_bridge`(#13) 없이도 지금 바로 동작합니다.

## 실행

아래 명령은 모두 컨테이너 안의 작업공간 루트(`/ws`)에서 실행합니다.

| 순서 | 명령 | 무엇을 하나요 | 왜 필요한가요 |
|---|---|---|---|
| 1 | `colcon build --symlink-install --packages-up-to pinky_parking_control` | 이 패키지를 빌드(설치)합니다 | 새 패키지라 처음 한 번은 빌드해야 `ros2`가 찾습니다 |
| 2 | `source install/setup.bash` | 방금 빌드한 것을 지금 터미널에 알립니다 | 안 하면 "패키지를 찾을 수 없음" 오류가 납니다 |
| 3 | `ros2 launch pinky_parking_bringup sim.launch.py` | 시뮬레이션(Gazebo, 로봇 두 대, Nav2)을 켭니다 | 명령을 받을 로봇이 있어야 합니다 |
| 4 | (다른 터미널, `./run.sh`로 들어가서) `ros2 launch pinky_parking_control control.launch.py` | 관제 서버를 켭니다 | API와 대시보드를 제공하는 프로그램입니다 |
| 5 | 호스트 PC 브라우저에서 `http://localhost:8080` | 대시보드를 엽니다 | `compose.yaml`이 컨테이너의 8080 포트를 호스트에 열어 둡니다 |

- API 설명서는 `http://localhost:8080/docs`에 있습니다. 화면에서 버튼을 눌러 명령을 직접 시험할 수도 있습니다.
- 서버 없이 화면만 보려면 `static/index.html`이 있는 폴더를 아무 웹 서버로 열고 주소 끝에 `?demo`를 붙입니다. 모의 로봇이 움직입니다(`?demo=lock`이면 비밀번호 `demo`가 걸린 화면).

## 팀 비밀번호와 다른 PC에서 열기

- 환경변수 `PINKY_CONTROL_PASSWORD`를 정하면 로봇을 **움직이는** 명령(이동, 주차, 출차, 주행)은 비밀번호가 있어야 보낼 수 있습니다. 보기와 **멈추는** 명령(작업 취소, 정지)은 비밀번호 없이 됩니다. 위험할 때 누구든 멈출 수 있어야 하기 때문입니다.
- 정하는 법, 같은 와이파이의 다른 PC에서 여는 법, 나중에 멀리 있는 팀원과 연결하는 법(Tailscale)은 [docs/remote_access.md](../docs/remote_access.md)에 있습니다.
- API로 직접 보낼 때는 머리글 `X-Control-Password: <비밀번호>`를 붙입니다. 틀리면 401이 돌아옵니다.

## 대시보드 사용법

| 하고 싶은 일 | 방법 |
|---|---|
| 로봇 고르기 | 오른쪽 카드에서 로봇 이름을 누릅니다. 이름 아래 파란 선이 생기고, 지도에서는 그 로봇 둘레에 파란 원이 생깁니다 |
| 원하는 곳으로 보내기 | 지도를 누르면 목표가 찍힙니다. 위쪽 막대의 **이동 보내기**를 누릅니다 |
| 주차·출차 | 카드에서 칸(`slot_10` 등)을 고르고 **주차** 또는 **출차**를 누릅니다 |
| 경로 주행 | 경로(`lap`)를 고르고 **주행**을 누릅니다 |
| 하던 일 그만두기 | **작업 취소**: Nav2 목표만 취소합니다 |
| 급히 세우기 | **정지**(로봇 하나) 또는 맨 위 **전체 정지**: 목표를 취소하고 속도 0 명령을 보냅니다 |

버튼이 회색이면 지금 누를 수 없는 상태입니다. 마우스를 올리면 이유가 나옵니다(예: "Nav2가 아직 켜지지 않았습니다").

## API (행동 단위)

| 단위 | 주소 | 보낼 내용 | 하는 일 |
|---|---|---|---|
| 기본 이동 | `POST /api/robots/{name}/goto` | `{"x": 0.3, "y": -1.1, "yaw": 0.0}` | map 좌표로 이동 (x, y [m], yaw [rad]) |
| | `POST /api/robots/{name}/cancel` | 없음 | 진행 중인 작업 취소 |
| | `POST /api/robots/{name}/stop` | 없음 | 취소하고 바로 멈춤 |
| | `POST /api/stop` | 없음 | 모든 로봇 정지 |
| 주차·출차 | `POST /api/robots/{name}/park` | `{"slot": "slot_10"}` | 주차 칸으로 이동 (`slots.yaml`) |
| | `POST /api/robots/{name}/unpark` | `{"slot": "slot_10"}` | 칸에서 나와 출차 위치로 이동 (`control.yaml`) |
| 주행 | `POST /api/robots/{name}/lap` | `{"route": "lap"}` | 경유점을 차례로 지남 (`route.yaml`) |
| 상태 조회 | `GET /api/robots`, `GET /api/robots/{name}` | | 위치, 속도, Nav2 상태, 지금 작업 |
| | `GET /api/log?limit=50` | | 끝난 작업 기록 (최근 것 먼저) |
| | `GET /api/layout`, `GET /api/map` | | 주차 칸·경로·출차 위치, 지도 |
| 팀 비밀번호 | `GET /api/auth`, `POST /api/auth/check` | 머리글 `X-Control-Password` | 비밀번호가 필요한지, 맞는지 |
| | `WS /api/ws` | | 위 상태를 초당 10번 보내 줌 |

**응답 코드 읽는 법**

| 코드 | 뜻 | 할 일 |
|---|---|---|
| 200 | 받았습니다 | 작업 번호와 목표가 돌아옵니다 |
| 401 | 팀 비밀번호가 틀렸거나 없습니다 | `X-Control-Password` 머리글을 확인합니다 |
| 404 | 없는 로봇·칸·경로 이름입니다 | 이름을 확인합니다. 오류 문장에 있는 이름 목록이 나옵니다 |
| 409 | 그 로봇이 이미 다른 작업 중입니다 | 먼저 `cancel`합니다 (한 로봇은 한 번에 작업 하나) |
| 422 | 보낸 내용이 잘못됐습니다 | x, y가 숫자인지 확인합니다 |
| 503 | 그 로봇의 Nav2가 아직 켜지지 않았습니다 | `sim.launch.py`가 Nav2를 15초 뒤에 켜므로 잠시 기다립니다 |

터미널에서 시험하는 예 (`curl`은 주소로 요청을 보내는 명령입니다):

```bash
curl http://localhost:8080/api/robots                     # 로봇 상태 보기
curl -X POST http://localhost:8080/api/robots/pinky1/park \
     -H 'Content-Type: application/json' -d '{"slot": "slot_10"}'   # pinky1 주차
curl -X POST http://localhost:8080/api/stop               # 전체 정지
```

## 설정

| 파일 | 내용 |
|---|---|
| `config/control.yaml` | 포트, 상태 전송 주기, 행동별 제한 시간, 출차 위치(임시) |
| `pinky_parking_bringup/config/robots.yaml` | 로봇 목록과 도메인 (그대로 읽음) |
| `pinky_parking_docking/config/slots.yaml` | 주차 칸 (그대로 읽음) |
| `pinky_parking_driving/config/route.yaml` | 경로 (그대로 읽음) |

- 출차 위치(`unpark_exits`)는 출차 명령(#9)이 생기기 전까지 쓰는 임시 값입니다. 공식 공장 월드에서 각 로봇의 시작 위치입니다.
- 실행 인자: `control_server --port 8081 --robots <파일> --slots <파일> --routes <파일> --map <지도 YAML>`

## 확인

| 명령 | 무엇을 확인하나요 |
|---|---|
| `colcon test --packages-select pinky_parking_control && colcon test-result` | 코드 스타일과 단위 시험(ROS 없이 도는 작업 상태·API 응답 시험) |
| `src/pinky_parking/scripts/control_smoke_test.sh` | 시뮬레이션을 켜고 **API로** pinky1 주차, pinky2 주행을 보내 둘 다 성공하는지. 마지막 줄 "CONTROL SMOKE TEST PASSED" |

## 파일

| 파일 | 하는 일 |
|---|---|
| `server.py` | HTTP·WebSocket 주소와 대시보드 제공, 실행 진입점 `control_server` |
| `fleet.py` | 행동 단위(goto, park, unpark, lap, cancel, stop)를 Nav2 목표로 바꿈, 제한 시간 감시 |
| `robot_link.py` | 로봇 한 대의 도메인에 rclpy로 붙어 Nav2 액션 호출, `amcl_pose`·`odom` 구독 |
| `model.py` | 작업 상태, 좌표 변환, 기록 (ROS 없이 시험 가능) |
| `mapdata.py` | Nav2 지도(YAML + PGM)를 대시보드용으로 읽음 |
| `static/` | 대시보드 (`index.html`, `app.css`, `app.js`, 모의 로봇 `demo.js`) |

## 안전

- 실물 로봇을 이 대시보드로 움직일 때는 다른 사람이 확인한 뒤 낮은 속도 제한으로 시험합니다.
- **정지**는 Nav2 목표를 취소하고 `cmd_vel`에 0을 한 번 보냅니다. 비상 정지 장치를 대신하지 않습니다.
