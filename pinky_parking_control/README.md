# pinky_parking_control

관제센터 패키지입니다. 기획(`docs/plan.md`)의 시연에 맞춰 두 로봇에게 **행동 단위**로 명령하는 API와, 게임 조종석처럼 다루는 **관제센터 화면**이 들어 있습니다.

## 한눈에 보기

```
 브라우저 (관제센터 화면)              관제 서버 (control_server, 프로세스 하나)
 ┌──────────────────────┐   명령      ┌──────────────────────────────────────┐
 │ 유닛 · 경기장 · 미션  │ ─────────▶ │ FastAPI + 시나리오 실행기               │
 │ 명령 막대 · 조이스틱  │ ◀───────── │   ├─ 도메인 11 연결 ── A pinky1 의 Nav2 │
 └──────────────────────┘  상태 10Hz  │   └─ 도메인 12 연결 ── B pinky2 의 Nav2 │
                                      └──────────────────────────────────────┘
```

| 용어 | 쉬운 뜻 |
|---|---|
| API | 프로그램끼리 주고받는 "주문서 양식". 화면 버튼도 결국 이 주문서를 보냅니다 |
| 유닛 A, B | 시연에서 부르는 이름. A = pinky1, B = pinky2 (`config/scenario.yaml`의 `callsigns`) |
| 차종 T00 · T10 · T15 | 트레일러 없음(전체 140 mm) · 10 cm(240 mm) · 15 cm(290 mm) |
| 도메인(`ROS_DOMAIN_ID`) | 로봇마다 따로 쓰는 "무전 채널". pinky1 = 11, pinky2 = 12 |
| Nav2 | 로봇이 목표까지 길을 찾아 가게 하는 ROS 2 주행 프로그램 |

**왜 이렇게 만들었나요?** Nav2 명령(액션)은 도메인을 건너가지 못합니다. 그래서 서버 하나가 로봇마다 그 로봇의 채널(도메인)에 직접 들어가 명령합니다. `domain_bridge`(#13) 없이도 지금 바로 동작합니다.

## 관제센터 여는 법

| 방법 | 언제 | 필요한 것 | 로봇 |
|---|---|---|---|
| **A. 데모로 화면만 보기** | 화면과 버튼을 눌러 보고 싶을 때 | Git, Python (Windows·Mac·Linux 모두) | 가짜 로봇 |
| **B. 시뮬레이션과 연결하기** | 진짜로 로봇을 움직여 보고 싶을 때 | Linux, Docker | 시뮬레이션 로봇 |

### A. 데모로 화면만 보기 (ROS 없어도 됨)

| 순서 | 명령 | 무엇을 하나요 | 왜 |
|---|---|---|---|
| 1 | `git clone https://github.com/rc10-tinyhaul/pinky_parking.git` | 저장소를 내 PC로 받습니다 | 처음 한 번만. 이미 받았으면 `git pull` |
| 2 | `cd pinky_parking` | 받은 폴더로 들어갑니다 | 다음 명령을 이 폴더에서 합니다 |
| 3 | `git switch henry/control-api` | 관제센터가 있는 브랜치로 바꿉니다 | PR #16이 `dev`에 합쳐지면 이 단계는 필요 없습니다 |
| 4 | `cd pinky_parking_control/static` | 화면 파일이 있는 폴더로 갑니다 | 이 폴더를 웹 페이지로 열어야 합니다 |
| 5 | `python -m http.server 8765` | 작은 웹 서버를 켭니다 | 브라우저가 화면 파일을 읽게 합니다. **창을 끄지 말고 둡니다** |
| 6 | 브라우저에서 `http://localhost:8765/?demo` | 관제센터가 열립니다 | `?demo`는 "가짜 로봇으로 보여 줘"라는 뜻입니다 |

- 다른 주소: `?demo=lock`은 팀 비밀번호 잠금 화면(비밀번호 `demo`)입니다.
- 끄기: 5번을 켠 창에서 `Ctrl + C`를 누릅니다.
- `python`이 없다고 나오면 [python.org](https://www.python.org/downloads/)에서 설치합니다. Mac·Linux에서는 `python3`으로 씁니다.
- `?demo` 없이 열면 진짜 서버를 찾느라 "연결 중"에 머뭅니다. 정상입니다.

### B. 시뮬레이션과 연결하기 (Linux + Docker)

| 순서 | 어디서 | 명령 | 왜 |
|---|---|---|---|
| 1 | 저장소 폴더 | `docker compose build` | 새 부품(FastAPI 등)을 설치합니다. 처음 한 번 |
| 2 | 저장소 폴더 | `./run.sh` | 시뮬레이션 컨테이너에 들어갑니다 |
| 3 | 컨테이너 안 | `colcon build --symlink-install --packages-up-to pinky_parking_control` | 관제 패키지를 설치합니다 |
| 4 | 컨테이너 안 | `source install/setup.bash` | 설치한 것을 지금 터미널에 알립니다 |
| 5 | 컨테이너 안 | `ros2 launch pinky_parking_bringup sim.launch.py` | 경기장에 시뮬레이션 로봇 두 대를 켭니다 |
| 6 | 새 터미널에서 `./run.sh` 후 | `source install/setup.bash` → `ros2 launch pinky_parking_control control.launch.py` | 관제 서버를 켭니다 |
| 7 | 그 PC 브라우저 | `http://localhost:8080` | 진짜 관제센터가 열립니다 (`?demo` 없이) |

#### 경기장 시뮬레이션

경기장 월드(`pinky_parking_gazebo/worlds/arena.world`)와 지도(`pinky_parking_navigation/maps/arena`)는 도면 좌표(`config/arena.yaml`)에서 만듭니다. 로봇 목록은 `pinky_parking_bringup/config/robots.yaml`이고, 월드 이름이 `arena.world`라 관제 화면이 자동으로 경기장이 됩니다.

- 테이프·주차 칸·주유구역은 바닥 그림이라 라이다에 안 보입니다. 위치 추정(AMCL)이 되도록 바닥 바깥에 낮은 벽과 블록을 둘렀습니다. 실물 경기장에는 없는 것입니다.
- `arena.yaml`을 고치면 `python3 scripts/make_arena_sim.py`로 월드와 지도를 다시 만들고 colcon build 합니다.
- 시뮬레이션을 끈 뒤 다시 켤 때 `gz sim`이 남아 있으면 시간(`/clock`)이 섞여 주행이 실패합니다. 컨테이너에서 `pgrep -af "gz sim"`으로 확인합니다.
- 같은 와이파이의 다른 PC에서 열거나 팀 비밀번호를 정하려면 [docs/remote_access.md](../docs/remote_access.md)를 봅니다.

### C. 실물 로봇과 연결하기

절차는 [docs/real_robot.md](../docs/real_robot.md)의 "5-1. 경기장 없이 좌표로만 시험"에 있습니다. 요약: 로봇을 주차 칸 자리(A = 11번, B = 10번)에 놓고 켠 뒤, PC에서 `real_check`(연결 점검) → `real.launch.py`(Nav2, 바퀴 기록으로 위치) → `control.launch.py` 순서로 켭니다.

- 한 대만 시험하려면 `real.launch.py`와 `control.launch.py`에 `only:=pinky1`을 붙입니다. 시나리오에서 연결하지 않은 로봇의 행동은 "B 없음"으로 건너뜁니다.
- 실물 로봇은 유닛 카드에 배터리 전압이 나옵니다.

## 경기장 배치

관제의 좌표는 모두 `config/arena.yaml`(팀 도면 2000 × 2000 mm를 옮긴 값)에서 옵니다. 시뮬레이션, 실물, 데모 화면이 같은 값을 씁니다.

경기장 도면 좌표 (원점 = 트랙 타원 중심, 단위 m, 반시계 주행):

| 이름 | 위치 | 비고 |
|---|---|---|
| 1번 트랙 (안쪽) | 중심선 반지름 0.405, 직선 0.6 | T00 |
| 2번 트랙 (바깥쪽) | 중심선 반지름 0.595 | T10 · T15 |
| 주차 칸 10, 11 | (0, −0.15), (0, +0.15), 190 × 300 mm | 마커 10, 11 |
| 주유구역 20 | (−0.995, −0.195), 190 × 350 mm | 마커 20 |
| 대기선 30 | y = 0.045, 대기 자리 (−0.995, 0.17) | 마커 30 |

## 행동 단위 API

| 단위 | 주소 | 보낼 내용 | 하는 일 |
|---|---|---|---|
| 기본 이동 | `POST /api/robots/{name}/goto` | `{"x", "y", "yaw"}` | 그 좌표로 이동 |
| | `POST /api/robots/{name}/teleop` | `{"linear", "angular"}` | 직접 운전. 0.5초 안에 다시 안 보내면 멈춤. 최고 0.15 m/s |
| | `POST /api/robots/{name}/cancel` · `/stop` · `POST /api/stop` | 없음 | 작업 취소 · 정지 · 전체 정지 |
| 주차·출차 | `POST /api/robots/{name}/park` · `/unpark` | `{"slot": "slot_10"}` | 칸으로 들어가기 · 칸에서 나와 트랙으로 |
| | `POST /api/robots/{name}/park/precise` | 없음 | **준비 중** (#7 마커 검출, #8 정밀 정렬). 지금은 501 |
| 주행 | `POST /api/robots/{name}/drive` | `{"track": 1}` | 경기장: 그 트랙을 반시계로 한 바퀴 (경유점 12개) |
| | `POST /api/robots/{name}/lane_change` | `{"to": 2}` | 경기장: 다른 트랙으로 옮겨 타기 |
| | `POST /api/robots/{name}/gap` | 없음 | **준비 중** (#12 앞뒤 간격 조절). 지금은 501 |
| 주유 | `POST /api/robots/{name}/fuel` | `{"step": "wait"}` | `wait` 대기선 30 → `enter` 주유구역 20 → `leave` 2번 트랙 복귀 |
| 차종 | `PUT /api/robots/{name}/vehicle` | `{"type": "T15"}` | 차종 표시를 바꿈 (회전 반경 반영은 #5, #6) |
| 시나리오 | `GET /api/scenario` | | 시연 단계와 진행 상태 |
| | `POST /api/scenario/next` · `/run` · `/stop` · `/confirm` · `/reset` | | 다음 단계 · 끝까지 · 멈춤 · 사람 확인 · 처음부터 |
| 상태 조회 | `GET /api/robots`, `/api/log`, `/api/layout`, `/api/map`, `WS /api/ws` | | 위치, 속도, 차종, 지금 작업, 기록, 배치 |

**응답 코드 읽는 법**

| 코드 | 뜻 | 할 일 |
|---|---|---|
| 200 | 받았습니다 | 작업 번호와 목표가 돌아옵니다 |
| 400 | 경기장에 없는 행동이나 값입니다 | 예: 없는 트랙 번호 |
| 401 | 팀 비밀번호가 틀렸거나 없습니다 | `X-Control-Password` 머리글 |
| 404 | 없는 로봇·칸·경로·차종 이름입니다 | 오류 문장에 있는 이름 목록 |
| 409 | 그 로봇이 다른 작업 중이거나 시나리오가 실행 중입니다 | 먼저 취소하거나 멈춥니다 |
| 422 | 보낸 내용이 잘못됐습니다 | 숫자, 단계 이름 확인 |
| 501 | 기획에는 있지만 아직 준비 중인 기능입니다 | 담당 이슈 번호가 함께 옵니다 |
| 503 | 그 로봇의 Nav2가 아직 켜지지 않았습니다 | 잠시 기다립니다 |

**지금 방식의 한계.** 트랙 주행과 차선 변경은 트랙 중심선 위 경유점을 Nav2에 주는 방식이라 도착 오차가 25 cm입니다. 정밀 차선 주행(#10, #11)이 생기면 API 주소는 그대로 두고 안쪽(`layout.py`, `fleet.py`)만 바꿉니다.

## 시연 시나리오

`config/scenario.yaml`에 기획의 "전체 테스트 시연 순서"를 단계로 적었습니다.

| 단계 | 내용 |
|---|---|
| 1 | 주차 구역에서 시작: A(T00) 11번, B(T10) 10번 |
| 2 | 1바퀴 직진: A 1번 트랙, B 2번 트랙 |
| 3 | 2번 트랙 간격 조절 1바퀴: **준비 중(#12)이라 건너뜀** |
| 4 | A 차선 변경 후 대기선 → 주유구역 |
| 5 | B 대기선에서 차례 기다리기 |
| 6 | A 트레일러 바꾸기: **사람 확인을 기다림** → T15 → 트랙 복귀 |
| 7 | B 주유 후 10번 칸에 주차 |
| 8 | A(T15) 2번 트랙 1바퀴 |
| 9 | A 11번 칸에 주차하고 종료 |

- 한 단계 안에서 A와 B는 동시에, 같은 로봇의 행동은 적힌 순서대로 합니다.
- **다음 단계**는 한 단계만, **끝까지**는 이어서 합니다. 실패하면 그 단계에서 멈추고 이유를 보여 줍니다.
- 단계를 고치려면 YAML만 바꿉니다. `do`: `park`, `unpark`, `drive`, `lane_change`, `fuel`, `goto`, `vehicle`, `confirm`. 아직 없는 기능은 `pending: 설명`.

## 관제센터 화면 조작

| 하고 싶은 일 | 마우스 | 키보드 |
|---|---|---|
| 유닛 고르기 | 유닛 카드나 경기장의 로봇을 클릭 | `1`, `2` |
| 그 자리로 보내기 | 경기장 빈 곳을 **우클릭** (휴대폰은 길게 누르기) | |
| 주차 칸에 주차 | 주차 칸을 우클릭, 또는 칸을 고르고 **주차** | `P` (출차 `U`) |
| 주유 | 대기선·주유구역을 우클릭, 또는 버튼 | `F` 대기선, `G` 주유구역, `H` 복귀 |
| 트랙 한 바퀴 | **1번 트랙**, **2번 트랙** | `Q`, `E` |
| 차선 변경 | **차선 변경** (지금 트랙의 반대편으로) | `C` |
| 차종 바꾸기 | 유닛 카드의 T00 · T10 · T15 | |
| 직접 운전 | **직접 운전**을 켜고 조이스틱을 끌기 | `M` 켜기/끄기, `W A S D` 또는 방향키 |
| 작업 취소 · 정지 | 버튼 | `X` · `Space` |
| 전체 정지 | 맨 위 **전체 정지** | `Esc` |
| 시나리오 | 오른쪽 **다음 단계 · 끝까지 · 멈춤 · 처음부터**, 확인 단계에서는 **확인했습니다** | |

- 우클릭하기 전에 마우스를 올리면 "우클릭: slot_10 주차"처럼 무엇을 할지 미리 보여 줍니다.
- 버튼이 회색이면 지금 할 수 없는 상태입니다. 마우스를 올리면 이유가 나옵니다(예: "작업 중입니다. 먼저 취소하세요").
- 직접 운전은 손을 떼거나 창을 벗어나면 0.5초 안에 섭니다. Nav2 작업 중에는 켜지지 않습니다.

## 실행

명령은 모두 컨테이너 안의 작업공간 루트(`/ws`)에서 실행합니다.

| 순서 | 명령 | 무엇을 하나요 | 왜 필요한가요 |
|---|---|---|---|
| 1 | `colcon build --symlink-install --packages-up-to pinky_parking_control` | 이 패키지를 설치합니다 | 새 파일이 생기면 한 번 빌드해야 `ros2`가 찾습니다 |
| 2 | `source install/setup.bash` | 설치한 것을 지금 터미널에 알립니다 | 안 하면 "패키지를 찾을 수 없음" |
| 3 | `ros2 launch pinky_parking_bringup sim.launch.py` | 시뮬레이션을 켭니다 | 명령을 받을 로봇이 있어야 합니다 |
| 4 | (다른 터미널) `ros2 launch pinky_parking_control control.launch.py` | 관제 서버를 켭니다 | API와 화면을 제공합니다 |
| 5 | 호스트 브라우저에서 `http://localhost:8080` | 관제센터를 엽니다 | `compose.yaml`이 8080을 열어 둡니다 |

- API 설명서: `http://localhost:8080/docs` (버튼을 눌러 직접 시험할 수 있습니다)
- 서버 없이 화면만 보려면 위의 "관제센터 여는 법 A"를 따릅니다.

## 팀 비밀번호와 다른 PC에서 열기

- 환경변수 `PINKY_CONTROL_PASSWORD`를 정하면 **움직이는** 명령은 비밀번호가 있어야 합니다. 보기와 **멈추는** 명령(취소, 정지, 시나리오 멈춤)은 비밀번호 없이 됩니다. 위험할 때 누구든 멈출 수 있어야 하기 때문입니다.
- 정하는 법과 다른 PC에서 여는 법은 [docs/remote_access.md](../docs/remote_access.md)에 있습니다.

## 설정

| 파일 | 내용 |
|---|---|
| `config/control.yaml` | 포트, 처음 차종, 직접 운전 속도 제한, 행동별 제한 시간 |
| `config/arena.yaml` | 경기장 도면 좌표: 트랙, 주차 칸, 출차 위치, 주유구역, 대기선, 차종 |
| `config/scenario.yaml` | 시연 단계와 유닛 이름(A, B) |
| `pinky_parking_bringup/config/robots.yaml` | 로봇 목록과 시작 자리 (그대로 읽음) |

`arena.yaml`은 경기장 좌표의 원본입니다. 경기장 월드와 지도(`scripts/make_arena_sim.py`), `park`·`lap` 명령의 칸·경로(`slots.yaml`, `route.yaml`)가 이 값을 따릅니다. 고치면 월드와 지도를 다시 만들고 두 파일도 맞춥니다.

## 확인

| 명령 | 무엇을 확인하나요 |
|---|---|
| `colcon test --packages-select pinky_parking_control && colcon test-result` | 코드 스타일과 단위 시험 (경기장 좌표, 행동, 시나리오 전체 흐름, API 응답, 비밀번호) |
| `src/pinky_parking/scripts/smoke_test.sh` | 경기장 시뮬레이션을 켜고 관제 API로 시연 시나리오 1·2단계(출차, 트랙 한 바퀴, 차선 변경)와 제자리 주차를 돌려 모두 성공하는지. 마지막 줄 "SMOKE TEST PASSED" |

## 파일

| 파일 | 하는 일 |
|---|---|
| `server.py` | HTTP·WebSocket 주소, 화면 제공, 실행 진입점 `control_server` |
| `fleet.py` | 행동 단위를 Nav2 목표로 바꿈, 차종, 직접 운전 제한, 시간 초과 감시 |
| `layout.py` | 경기장 도면 계산(트랙 경유점, 차선 변경, 주차·주유 위치) |
| `scenario.py` | 시연 시나리오 실행기 |
| `robot_link.py` | 로봇 한 대의 도메인에 rclpy로 붙어 Nav2 액션 호출, 위치·속도 구독, 직접 운전 |
| `model.py`, `mapdata.py` | 작업 상태와 기록, Nav2 지도 읽기 |
| `static/` | 관제센터 화면 (`index.html`, `app.css`, `app.js`, 도면 계산 `arena.js`, 모의 로봇 `demo.js`) |

## 안전

- 실물 로봇은 다른 사람이 확인한 뒤 낮은 속도 제한(`control.yaml`의 `teleop`)으로 시험합니다.
- **정지**는 Nav2 목표를 취소하고 `cmd_vel`에 0을 보냅니다. 비상 정지 장치를 대신하지 않습니다.
