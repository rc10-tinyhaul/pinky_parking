# pinky_parking

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)
![License: Apache-2.0](https://img.shields.io/badge/License-Apache%202.0-blue)

두산 로봇팔, OMX, Pinky Pro를 쓰는 프로젝트 중 Pinky Pro 자율주행(주행·주차) 저장소입니다. 전체 기획은 [docs/plan.md](docs/plan.md)에 있습니다.

## 기능

- 경기장 월드(2 × 2 m, 2차선 트랙, 주차 칸, 주유구역)에서 Pinky Pro 두 대 동시 시뮬레이션
- 로봇별 `ROS_DOMAIN_ID`(pinky1 = 11, pinky2 = 12)와 Nav2
- 명령: `send_goal`(목표 전송), `auto_explore`(지도 작성용 탐색), `sim_speed`(시뮬레이션 속도 측정)
- 관제센터: 행동 단위 API, 시연 시나리오 실행, 게임 조종석 방식 웹 화면 (`pinky_parking_control`, http://localhost:8080)
- 동작 검사 스크립트 (`scripts/smoke_test.sh`: 경기장에서 출차, 트랙 주행, 차선 변경, 주차)
- 실물 로봇 두 대 연결과 실행 (`scripts/real_up.sh`, [docs/real_robot.md](docs/real_robot.md))
- Docker 환경 (GPU 없음 / NVIDIA / Intel·AMD)

## 요구 사항

| 방식 | 요구 사항 |
|---|---|
| Docker | Linux, Docker Engine, Docker Compose. NVIDIA GPU를 쓰려면 [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) |
| 소스 빌드 | Ubuntu 24.04, ROS 2 Jazzy Desktop |

의존 저장소: [pinklab-art/pinky_pro](https://github.com/pinklab-art/pinky_pro) (`pinky_parking.repos`에 커밋 고정)

## 설치

### Docker

```bash
git clone https://github.com/rc10-tinyhaul/pinky_parking.git
cd pinky_parking
./run.sh
```

처음 실행하면 이미지를 빌드합니다(수 분).

### 소스 빌드

```bash
sudo apt update && sudo apt install ros-dev-tools
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/rc10-tinyhaul/pinky_parking.git
vcs import < pinky_parking/pinky_parking.repos

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init                                       # 처음 한 번만
rosdep update
rosdep install --from-paths src/pinky_parking src/pinky_pro/pinky_description \
  src/pinky_pro/pinky_gz_sim src/pinky_pro/pinky_navigation --ignore-src -y
colcon build --symlink-install --packages-up-to $(colcon list --base-paths src/pinky_parking --names-only)
source install/setup.bash
```

## 사용법

### 컨테이너 (호스트의 저장소 폴더에서)

```bash
./run.sh          # 컨테이너를 띄우고 셸로 들어감. 터미널마다 실행
./run.sh down     # 종료
./run.sh check    # 감지한 그래픽과 고를 설정 확인
```

`run.sh`는 그래픽(NVIDIA / Intel·AMD / 없음)을 감지해 알맞은 compose 설정으로 실행합니다. 컨테이너는 호스트 사용자와 같은 UID의 일반 사용자(`ubuntu`)로 실행됩니다. NVIDIA GPU가 있는데 NVIDIA Container Toolkit이 없으면 설치할지 묻습니다(`scripts/install_nvidia_toolkit.sh`).

그래픽을 직접 고르려면 `PINKY_GPU=nvidia|intel|cpu ./run.sh`로 실행합니다.

실물 로봇과 통신하려면 `PINKY_NETWORK=host ./run.sh`로 호스트 네트워크를 씁니다(`compose.real.yaml`). 이때는 시뮬레이션을 띄우지 않습니다. 같은 `ROS_DOMAIN_ID`의 실물 로봇과 섞이기 때문입니다. 네트워크를 바꾸려면 먼저 `./run.sh down`을 합니다.

아래 명령은 작업공간 루트(컨테이너에서는 `/ws`)에서 실행합니다.

### 동작 확인

```bash
src/pinky_parking/scripts/smoke_test.sh   # 화면 없이 경기장 사이클(약 3분). 성공하면 "SMOKE TEST PASSED"
colcon test --packages-select $(colcon list --base-paths src/pinky_parking --names-only) && colcon test-result
```

### 기본 사이클

```bash
ros2 launch pinky_parking_bringup sim.launch.py               # Gazebo + 두 대 + Nav2 (GPU 없으면 camera:=false)

ROS_DOMAIN_ID=11 ros2 launch pinky_navigation gz_nav2_view.launch.xml   # pinky1 RViz
ROS_DOMAIN_ID=12 ros2 launch pinky_navigation gz_nav2_view.launch.xml   # pinky2 RViz

ROS_DOMAIN_ID=11 ros2 run pinky_parking_docking park slot_10   # pinky1 → 주차 칸
ROS_DOMAIN_ID=12 ros2 run pinky_parking_driving lap            # pinky2 → 경유점 주행
ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup send_goal 0.3 -1.1 0.0   # 임의 목표 (x, y, yaw)
```

`ROS_DOMAIN_ID`로 대상 로봇을 고릅니다. 각 패키지의 인자와 설정은 패키지 README를 참고하세요.

### 실물 로봇

절차는 [docs/real_robot.md](docs/real_robot.md)에 있습니다. 로봇에서 공식 bringup과 속도 명령 감시(`cmd_vel_watchdog`)를 띄우고, PC에서는 `PINKY_NETWORK=host ./run.sh`로 들어가 Nav2를 `use_sim_time:=false`로 실행합니다. 첫 시험은 `max_speed:=0.1` 이하로 합니다.

### 문제 해결

| 증상 | 조치 |
|---|---|
| 시뮬레이션이 느림 | `camera:=false`로 실행합니다 |
| RViz 첫 화면이 1분 넘게 걸림 | GPU 없이 CPU로 그리기 때문입니다. `./run.sh check`로 감지된 그래픽을 확인합니다 |
| 창이 뜨지 않음 | 호스트에서 `xhost +si:localuser:$USER`를 실행합니다 |
| `docker 에 접속할 수 없습니다` | `sudo usermod -aG docker $USER` 후 다시 로그인합니다 |

## 구조

| 패키지 | 내용 |
|---|---|
| [pinky_parking_description](pinky_parking_description/README.md) | 차량 모델(기본, 트레일러) |
| [pinky_parking_gazebo](pinky_parking_gazebo/README.md) | Gazebo 월드, 로봇 스폰과 브리지 |
| [pinky_parking_navigation](pinky_parking_navigation/README.md) | 로봇별 Nav2, 지도 |
| [pinky_parking_bringup](pinky_parking_bringup/README.md) | 로봇 목록, 전체 실행 launch, 도구 명령 |
| [pinky_parking_docking](pinky_parking_docking/README.md) | 입차·출차 |
| [pinky_parking_driving](pinky_parking_driving/README.md) | 도로 주행 |
| [pinky_parking_control](pinky_parking_control/README.md) | 관제센터: 행동 단위 API, 시연 시나리오, 웹 화면 |

```
pinky_parking/
├── docs/                  전체 기획, 경기장 도면, 실물 로봇 운용
├── pinky_parking_*/       ROS 2 패키지 (위 표)
├── docker/                Dockerfile, entrypoint
├── run.sh                 Docker 실행 (그래픽 자동 감지)
├── scripts/               동작 확인, 실물 실행, 경기장 월드·지도와 마커 생성, NVIDIA Container Toolkit 설치
├── compose.yaml           Docker Compose 설정
├── compose.gpu.yaml       NVIDIA GPU 설정
├── compose.real.yaml      실물 로봇용 호스트 네트워크 설정
└── pinky_parking.repos    의존 저장소 (pinky_pro)
```

## 기여

[CONTRIBUTING.md](CONTRIBUTING.md)를 참고하세요.

## 로드맵

- [x] 두 대 동시 시뮬레이션, 로봇별 Nav2
- [x] Docker 환경
- [x] 관제센터 (행동 단위 API, 시연 시나리오, 웹 화면)
- [x] 실물 두 대로 시연 시나리오 실행 (간격 조절 단계 제외)
- [x] 경기장 월드 (트랙, 주차 칸, 주유구역)
- [ ] 경기장 마커 (ArUco)
- [x] 차량 길이 2종 (기본 / 트레일러 10 cm)
- [x] 출차, 주차 칸 진입
- [ ] 주차 칸 정밀 정렬 (마커 기준, 반복 정지 오차 기록)
- [x] 트랙 주행, 차선 변경
- [ ] 앞뒤 간격 조절

## 라이선스

[Apache License 2.0](LICENSE)
