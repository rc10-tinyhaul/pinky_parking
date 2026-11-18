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
```

- 코드를 바꾸면 `colcon test`와 smoke test를 통과시킨 뒤 커밋합니다.
- 파이썬 코드 수정은 바로 반영됩니다. 새 파일(launch, config, 월드)이나 새 실행 파일(`setup.py`의 `console_scripts`)을 추가했으면 colcon build를 다시 합니다.

## 패키지와 범위

작업은 해당 패키지 안에서 합니다. 다른 패키지를 고쳐야 하면 PR 설명에 이유를 적습니다.

| 패키지 | 맡는 일 | 다른 패키지와 주고받는 것 |
|---|---|---|
| `pinky_parking_gazebo` | 월드(경기장), 로봇 스폰, 브리지 | `world.launch.py`, `spawn_robot.launch.py`, `worlds/`, 각 도메인의 `/scan` `/odom` `/cmd_vel` `/tf` `/camera/*` |
| `pinky_parking_navigation` | 로봇별 Nav2, 지도, Nav2 파라미터 | `robot_nav.launch.py`, `maps/`, 각 도메인의 Nav2 액션(`navigate_to_pose`, `follow_waypoints`) |
| `pinky_parking_bringup` | 로봇 목록, 전체 실행 launch, 도구 명령, 통합 | `config/robots.yaml`, `launch/sim.launch.py` |
| `pinky_parking_docking` | 입차·출차 | `park` 명령, `config/slots.yaml` |
| `pinky_parking_driving` | 도로 주행(차선, 차선 변경, 간격 조절) | `lap` 명령, `config/route.yaml` |

`gazebo`와 `navigation`의 launch는 로봇 한 대 단위로 인자를 받고, `bringup`이 로봇 목록을 읽어 로봇마다 호출합니다.

## 구조와 설계

- 로봇마다 `ROS_DOMAIN_ID`를 나눕니다(pinky1 = 11, pinky2 = 12, 관제 PC = 0 예정). 각 도메인의 토픽은 로봇 한 대일 때와 같은 이름(`/scan`, `/odom`, `/cmd_vel`)이라 공식 `pinky_navigation` launch를 그대로 씁니다. 실물도 같은 구조로 운용합니다.
- 도메인 사이는 `domain_bridge`로 잇습니다(예정). 액션(Nav2 `navigate_to_pose`)과 `/tf`는 도메인을 건너지 못하므로, 목표 전송은 그 로봇의 도메인 안에서 하고 관제 PC는 map 좌표 상태 토픽을 받습니다.
- [pinky_pro](https://github.com/pinklab-art/pinky_pro)(로봇 모델, 월드, Nav2 설정)는 의존성입니다. `pinky_parking.repos`에 고정한 버전을 `vcs import`로 받아 빌드하며, 이 저장소에는 들어 있지 않습니다. 받아 온 파일을 고쳐도 저장소에 남지 않으므로, 동작을 바꿀 때는 이 저장소의 코드에서 덮어씁니다(예: `pinky_parking_gazebo`의 `sim.sim_urdf`, `robot_nav.launch.py`). 수정이 커지면 조직에 포크하고 `.repos`의 주소를 바꿉니다.

## 알아 둘 것

- 공식 URDF에 namespace를 주면 Gazebo에서 센서가 사라집니다. `pinky_parking_gazebo`의 `sim.sim_urdf`가 보정합니다.
- 실물 `pinky_bringup`에는 namespace 인자가 없습니다(그래서 도메인으로 나눕니다).
- LiDAR 프레임은 로봇 정면 기준 180° 돌아 있습니다.
- Nav2 도착 허용 오차는 0.25 m입니다. 경기장 차선의 빈 공간은 170 mm(차선 폭 190 mm, 테이프 20 mm)라, 차선 주행과 주차에는 별도 정밀화(카메라 차선 추종, 미리 만든 경로를 `FollowPath`로 주기, 차선 마스크 등)가 필요할 것으로 봅니다. 방식은 아직 정하지 않았습니다.
- `nav2_simple_commander`의 `BasicNavigator.waitUntilNav2Active()`는 기본값으로 초기 위치 (0, 0)을 보내 AMCL 위치를 덮어씁니다. `localizer='robot_localization'`을 줍니다(`park.py` 참고).
- 파이썬 가상환경(`.venv`)이 켜진 셸에서는 colcon 빌드가 실패합니다(`catkin_pkg` 없음).
- Intel + NVIDIA 노트북에서 `/dev/dri` 전체를 컨테이너에 넘기면 화면이 깨집니다. `run.sh`가 알아서 고릅니다.
- 실물 로봇을 움직이는 명령(bringup, 목표 전송, 주행)은 사람이 확인한 뒤 실행합니다. 기본은 시뮬레이션입니다.
