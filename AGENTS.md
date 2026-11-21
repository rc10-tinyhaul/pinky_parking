# AGENTS.md

AI 코딩 도구(Claude Code, Codex, Cursor 등)를 위한 작업 지침입니다. 브랜치, 커밋, PR, 코드 규칙은 [CONTRIBUTING.md](CONTRIBUTING.md)를 따릅니다.

## 프로젝트

- 두산 로봇팔, OMX, Pinky Pro를 쓰는 프로젝트 중 **Pinky Pro 자율주행(주행·주차)** 저장소입니다. 전체 기획은 `docs/plan.md`입니다.
- 이 저장소의 범위는 기획의 주행 부분입니다: 경기장 월드(트랙, 주차 칸), 차량 길이 2종(기본 / 트레일러 10 cm), 입차·출차, 도로 주행(직진, 곡선, 차선 변경, 앞뒤 간격 조절). 진행 상황은 README의 로드맵을 봅니다.
- ROS 2 Jazzy, Gazebo Harmonic, Nav2, Ubuntu 24.04.

## 실행과 확인

```bash
./run.sh                                   # 호스트에서. 그래픽 감지 후 Docker 컨테이너 셸
# 아래는 작업공간 루트(컨테이너에서는 /ws)에서
colcon build --symlink-install --packages-up-to $(colcon list --base-paths src/pinky_parking --names-only)
colcon test --packages-select $(colcon list --base-paths src/pinky_parking --names-only) && colcon test-result
src/pinky_parking/scripts/smoke_test.sh    # 기본 사이클(park, lap) 검사, "SMOKE TEST PASSED"
src/pinky_parking/scripts/control_smoke_test.sh   # 관제 API 검사, "CONTROL SMOKE TEST PASSED"
```

- 코드를 바꾸면 `colcon test`와 smoke test를 통과시킨 뒤 커밋합니다. 관제(`pinky_parking_control`)나 그것이 쓰는 Nav2 설정을 바꿨으면 `control_smoke_test.sh`도 돌립니다.
- 파이썬 코드 수정은 바로 반영됩니다. 새 파일(launch, config, 월드)이나 새 실행 파일(`setup.py`의 `console_scripts`)을 추가했으면 colcon build를 다시 합니다.
- `colcon test`는 코드 스타일 검사(`ament_flake8`, `ament_pep257`)뿐입니다. 동작은 smoke test가 확인합니다. 패키지 하나만 검사하려면:

  ```bash
  colcon test --packages-select pinky_parking_docking --event-handlers console_direct+
  ```

- `./run.sh`는 대화형 셸을 엽니다. 셸 없이 호스트에서 명령 하나를 실행하려면(AI 도구 등) 컨테이너를 띄운 뒤 `exec`를 씁니다. `exec`는 ROS 환경을 불러오지 않으므로 직접 source 합니다(smoke test는 스스로 불러옵니다).

  ```bash
  ./run.sh up
  docker compose exec -T sim bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon test --packages-select pinky_parking_docking'
  docker compose exec -T sim src/pinky_parking/scripts/smoke_test.sh
  ```

- smoke test는 시뮬레이션을 직접 띄우므로, 같은 컨테이너에 다른 시뮬레이션이 떠 있으면 먼저 끕니다. 실패하면 `park=`, `lap=` 뒤의 종료 코드(1 = 인자 오류, 2 = 시간 초과, 3 = Nav2 실패)와 로그(`/tmp/pinky_smoke/sim.log`)를 봅니다. 환경변수 `CAMERA`, `GOAL_TIMEOUT`, `LOG_DIR`로 조정합니다.

## Docker 환경

- 저장소는 컨테이너의 `/ws/src/pinky_parking`에 연결되어 있어 고친 코드와 저장한 지도는 호스트에 남습니다. `/ws/build`, `/ws/install`은 컨테이너 안에만 있어 `./run.sh down` 뒤에는 이미지를 빌드했을 때의 상태로 돌아갑니다. 새 파일·실행 파일을 추가한 뒤 컨테이너를 새로 띄웠으면 colcon build를 다시 합니다.
- `run.sh`는 이미지가 없을 때만 빌드합니다. `docker/Dockerfile`, `package.xml`의 의존성, `pinky_parking.repos`를 바꿨으면 `docker compose build`를 직접 실행합니다.
- 컨테이너는 호스트와 네트워크를 공유하지 않습니다(같은 네트워크의 실물 로봇과 섞이지 않게). 호스트의 `ros2` 명령으로는 시뮬레이션 토픽이 보이지 않으므로, 모든 터미널은 `./run.sh`로 같은 컨테이너에 들어갑니다.
- 실물 로봇용은 `PINKY_NETWORK=host ./run.sh`(`compose.real.yaml`, 호스트 네트워크)입니다. 이 모드에서는 시뮬레이션을 띄우지 않습니다.

## 패키지와 범위

작업은 해당 패키지 안에서 합니다. 다른 패키지를 고쳐야 하면 PR 설명에 이유를 적습니다.

| 패키지 | 맡는 일 | 다른 패키지와 주고받는 것 |
|---|---|---|
| `pinky_parking_gazebo` | 월드(경기장), 로봇 스폰, 브리지 | `world.launch.py`, `spawn_robot.launch.py`, `worlds/`, 각 도메인의 `/scan` `/odom` `/cmd_vel` `/tf` `/camera/*` |
| `pinky_parking_navigation` | 로봇별 Nav2, 지도, Nav2 파라미터 | `robot_nav.launch.py`, `maps/`, 각 도메인의 Nav2 액션(`navigate_to_pose`, `follow_waypoints`) |
| `pinky_parking_bringup` | 로봇 목록, 전체 실행 launch, 도구 명령, 통합 | `config/robots.yaml`, `launch/sim.launch.py` |
| `pinky_parking_docking` | 입차·출차 | `park` 명령, `config/slots.yaml` |
| `pinky_parking_driving` | 도로 주행(차선, 차선 변경, 간격 조절) | `lap` 명령, `config/route.yaml` |
| `pinky_parking_control` | 관제센터: 행동 단위 API, 시연 시나리오, 웹 화면, 경기장 좌표 | `control.launch.py`, `config/arena.yaml`, `config/scenario.yaml`, HTTP API(포트 8080) |

`gazebo`와 `navigation`의 launch는 로봇 한 대 단위로 인자를 받고, `bringup`이 로봇 목록을 읽어 로봇마다 호출합니다.

경기장의 주차·출차·트랙 주행·차선 변경은 `control`이 Nav2 목표와 경로(`FollowPath`)로 직접 보냅니다. `docking`의 `park`와 `driving`의 `lap`은 공장 월드 좌표의 명령으로, smoke test가 기본 사이클 검사에 씁니다.

## 구조와 설계

- 로봇마다 `ROS_DOMAIN_ID`를 나눕니다(pinky1 = 11, pinky2 = 12). 각 도메인의 토픽은 로봇 한 대일 때와 같은 이름(`/scan`, `/odom`, `/cmd_vel`)이라 공식 `pinky_navigation` launch를 그대로 씁니다. 실물도 같은 구조로 운용합니다.
- Gazebo는 하나이고 ROS 도메인과 무관합니다. Gazebo 안에서만 토픽에 `<name>/` 접두어를 붙여 로봇을 구분하고(`pinky1/scan`), 로봇마다 그 도메인에서 실행한 `ros_gz_bridge`가 접두어를 떼어 넘깁니다(`pinky_parking_gazebo/sim.py`의 `bridge_entries`). 브리지할 토픽을 추가할 때는 여기에 넣습니다.
- 로봇 대수·도메인·위치는 로봇 목록 파일(`pinky_parking_bringup/config/robots.yaml`, 임시 경기장은 `robots_arena.yaml`)에서만 정합니다. 위치는 Gazebo 스폰 위치이자 AMCL 초기 위치로 함께 쓰입니다(지도를 월드 원점에서 만들어 지도 좌표 = 월드 좌표).
- 다른 도메인에서 노드를 띄우는 방법은 두 가지입니다. 직접 만든 `Node`에는 `additional_env`로 `ROS_DOMAIN_ID`를 줍니다(`spawn_robot.launch.py`). 남의 launch 파일을 통째로 띄울 때는 포함(include)하면 도메인이 적용되지 않으므로, 별도 `ros2 launch` 프로세스로 실행합니다(`robot_nav.launch.py`).
- Nav2 파라미터는 공식 `nav2_params.yaml`을 읽어 필요한 값만 바꾼 임시 파일로 넘깁니다(`pinky_parking_navigation/params.py`). Nav2 파라미터를 바꿀 때는 이 함수에서 덮어씁니다.
- `sim.launch.py`는 Gazebo와 로봇을 띄운 뒤 `nav_delay`(15초) 뒤에 Nav2를 시작합니다. 명령(`park`, `lap`)은 Nav2가 켜질 때까지 스스로 기다립니다.
- 액션(Nav2 `navigate_to_pose`)과 `/tf`는 도메인을 건너지 못합니다. 그래서 관제 서버는 로봇마다 그 도메인의 rclpy `Context`를 따로 만들어 한 프로세스에서 모든 로봇 도메인에 직접 붙습니다(`pinky_parking_control/robot_link.py`). 위치(`amcl_pose`)를 읽고 Nav2 액션을 보냅니다. `domain_bridge`와 관제용 도메인은 쓰지 않습니다.
- 실물은 로봇 시계가 PC와 틀려 로봇을 도메인 + 100(pinky1 = 111)에서 켜고, PC의 `clock_bridge`가 시각을 고쳐 원래 도메인(11)으로 넘깁니다(`real.launch.py`의 `bridge`). 절차는 `docs/real_robot.md`입니다.
- [pinky_pro](https://github.com/pinklab-art/pinky_pro)(로봇 모델, 월드, Nav2 설정)는 의존성입니다. `pinky_parking.repos`에 고정한 버전을 `vcs import`로 받아 빌드하며, 이 저장소에는 들어 있지 않습니다. 받아 온 파일을 고쳐도 저장소에 남지 않으므로, 동작을 바꿀 때는 이 저장소의 코드에서 덮어씁니다(예: `pinky_parking_gazebo`의 `sim.sim_urdf`, `robot_nav.launch.py`). 수정이 커지면 조직에 포크하고 `.repos`의 주소를 바꿉니다.

## 알아 둘 것

- 공식 URDF에 namespace를 주면 Gazebo에서 센서가 사라집니다. `pinky_parking_gazebo`의 `sim.sim_urdf`가 보정합니다.
- 실물 `pinky_bringup`에는 namespace 인자가 없습니다(그래서 도메인으로 나눕니다).
- LiDAR 프레임은 로봇 정면 기준 180° 돌아 있습니다.
- Nav2 도착 허용 오차는 0.25 m입니다. 경기장 차선의 빈 공간은 170 mm(차선 폭 190 mm, 테이프 20 mm)라, 관제는 위치를 바퀴 기록 + 라이다 고정벽 지도(AMCL) + ArUco 마커로 잡고, 트랙은 차선 중심선 경로를 `FollowPath`로 주며, 주차·출차는 정밀 도착 기준(3 cm, 약 5°)을 씁니다(`robot_nav.launch.py`의 `localization`, `goal_tolerance`, `fine_amcl`).
- `nav2_simple_commander`의 `BasicNavigator.waitUntilNav2Active()`는 기본값으로 초기 위치 (0, 0)을 보내 AMCL 위치를 덮어씁니다. `localizer='robot_localization'`을 줍니다(`park.py` 참고).
- 지금의 주차 칸(`slots.yaml`)과 경로(`route.yaml`) 좌표, 지도(`maps/factory`)는 공식 공장 월드 기준의 임시 값입니다. 경기장 좌표는 `pinky_parking_control/config/arena.yaml`, 임시 경기장 시뮬레이션은 `robots_arena.yaml` + `maps/arena.yaml`(`scripts/make_arena_sim.py`가 생성)입니다. 경기장 월드(#1)가 정해지면 기본 설정을 옮깁니다(#4). smoke test가 `slot_10`과 `lap` 경로를 쓰므로 이름을 바꾸면 `scripts/smoke_test.sh`도 고칩니다.
- 지도는 로봇 한 대만 띄워 만듭니다(`pinky_parking_navigation/README.md`의 "지도 만들기").
- 파이썬 가상환경(`.venv`)이 켜진 셸에서는 colcon 빌드가 실패합니다(`catkin_pkg` 없음).
- Intel + NVIDIA 노트북에서 `/dev/dri` 전체를 컨테이너에 넘기면 화면이 깨집니다. `run.sh`가 알아서 고릅니다.
- 실물 로봇을 움직이는 명령(bringup, 목표 전송, 주행)은 사람이 확인한 뒤 실행합니다. 기본은 시뮬레이션입니다. 실물 절차는 `docs/real_robot.md`입니다.
- 공식 구동 노드는 속도 명령 시간 제한이 없어, 명령이 끊기면 실물 로봇이 마지막 속도로 계속 달립니다. 실물에서는 `cmd_vel_watchdog`를 로봇에서 함께 띄웁니다. 실물에 쓸 명령을 만들 때는 주행 중 속도 명령을 계속 보내고, 끝날 때 0을 보냅니다.
