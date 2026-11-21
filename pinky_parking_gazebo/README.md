# pinky_parking_gazebo

Gazebo 월드와 로봇 스폰입니다. 로봇의 ROS 쪽 노드는 로봇마다 다른 `ROS_DOMAIN_ID`에서 실행합니다.

## Launch 파일

### `world.launch.py`

Gazebo 월드를 띄웁니다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `world_name` | `arena.world` | 월드 파일 이름. 이 패키지 `worlds/`를 먼저 찾고, 없으면 `pinky_gz_sim/worlds` |
| `gui` | `true` | Gazebo 창 |
| `headless` | `false` | 화면 없는 환경용 센서 렌더링 (`gui:=false`와 함께) |

### `spawn_robot.launch.py`

실행 중인 월드에 Pinky Pro 한 대를 띄우고, `domain_id` 도메인에 `robot_state_publisher`, `joint_state_publisher`, `ros_gz_bridge`를 실행합니다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `name` | (필수) | 로봇 이름. Gazebo 토픽 접두어(`pinky1/scan`)로 씁니다 |
| `domain_id` | (필수) | 이 로봇의 `ROS_DOMAIN_ID` |
| `x`, `y`, `yaw` | `0.0` | 스폰 위치 [m], 방향 [rad] |
| `camera` | `true` | 카메라 영상 브리지와 시뮬레이션 카메라 서버(`sim_camera`, 포트 8000 + 도메인) |
| `cam_tilt_deg` | `8` | 카메라 기울기 [deg] |
| `vehicle` | `T00` | 차종(`pinky_parking_description`의 `config/vehicles.yaml`) |

## 토픽 (각 로봇의 도메인)

| 토픽 | 방향 |
|---|---|
| `/clock`, `/tf`(`odom → base_footprint`), `/scan`, `/odom`, `/joint_states` | Gazebo → ROS |
| `/cmd_vel` | ROS → Gazebo |
| `/camera/image_raw`, `/camera/camera_info` | Gazebo → ROS (`camera:=true`) |

LiDAR 프레임은 로봇 정면 기준 180° 돌아 있습니다.

## 경기장 월드와 마커

`worlds/arena.world`와 마커 모델 `models/aruco_<번호>`는 `scripts/make_arena_sim.py`가 `pinky_parking_control/config/arena.yaml`에서 만듭니다. 직접 고치지 않습니다. 마커는 바닥에 붙인 ArUco 그림(`DICT_4X4_50`, 검은 정사각형 50 mm)으로 10(두 칸 경계), 11(11번 칸 위 끝), 20(주유구역), 30(대기선)입니다. `world.launch.py`가 `models/`를 `GZ_SIM_RESOURCE_PATH`에 넣습니다.

## 실행 파일

| 명령 | 설명 |
|---|---|
| `marker_check [--seconds S] [--save 파일.png]` | 그 도메인의 `/camera/image_raw`(`camera:=true`)를 S초(기본 3) 받아 보이는 마커 번호와 화면 위치를 출력합니다. 못 찾으면 종료 코드 1 |
| `sim_camera` | 시뮬레이션 카메라 서버. 실물 로봇의 `scripts/pinky_camera.py`와 같은 `/markers`(JSON), `/snapshot.jpg`, `/stream`을 냅니다. `/markers`에는 Gazebo 카메라 값(`camera`: 초점 거리, 화면 중심, 앞쪽 거리)을 더 싣습니다. `spawn_robot.launch.py`가 `camera:=true`일 때 포트 8000 + 도메인(pinky1 = 8011)으로 띄웁니다 |

```bash
ROS_DOMAIN_ID=11 ros2 run pinky_parking_gazebo marker_check
```

시뮬레이션 카메라(높이 6.3 cm, 아래로 8°)는 로봇 중심에서 앞으로 약 0.20 ~ 0.40 m에 있는 바닥 마커를 찾습니다. 더 가까우면 화면 아래로 벗어나고, 더 멀면 너무 납작해집니다. 시작 자리의 A는 11번 마커가 너무 가까워(0.15 m) 보이지 않습니다.

## 월드 추가

`worlds/`에 `.sdf`(또는 `.world`) 파일을 두고 다시 빌드한 뒤, `world_name:=<파일 이름>`으로 실행합니다.
