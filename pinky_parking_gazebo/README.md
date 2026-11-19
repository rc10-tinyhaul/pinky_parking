# pinky_parking_gazebo

Gazebo 월드와 로봇 스폰입니다. 로봇의 ROS 쪽 노드는 로봇마다 다른 `ROS_DOMAIN_ID`에서 실행합니다.

## Launch 파일

### `world.launch.py`

Gazebo 월드를 띄웁니다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `world_name` | `pinky_factory.world` | 월드 파일 이름. 이 패키지 `worlds/`를 먼저 찾고, 없으면 `pinky_gz_sim/worlds` |
| `gui` | `true` | Gazebo 창 |
| `headless` | `false` | 화면 없는 환경용 센서 렌더링 (`gui:=false`와 함께) |

### `spawn_robot.launch.py`

실행 중인 월드에 Pinky Pro 한 대를 띄우고, `domain_id` 도메인에 `robot_state_publisher`, `joint_state_publisher`, `ros_gz_bridge`를 실행합니다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `name` | (필수) | 로봇 이름. Gazebo 토픽 접두어(`pinky1/scan`)로 씁니다 |
| `domain_id` | (필수) | 이 로봇의 `ROS_DOMAIN_ID` |
| `x`, `y`, `yaw` | `0.0` | 스폰 위치 [m], 방향 [rad] |
| `camera` | `true` | 카메라 영상 브리지 |
| `cam_tilt_deg` | `8` | 카메라 기울기 [deg] |
| `vehicle` | `basic` | 차종(`pinky_parking_description`의 `config/vehicles.yaml`) |

## 토픽 (각 로봇의 도메인)

| 토픽 | 방향 |
|---|---|
| `/clock`, `/tf`(`odom → base_footprint`), `/scan`, `/odom`, `/joint_states` | Gazebo → ROS |
| `/cmd_vel` | ROS → Gazebo |
| `/camera/image_raw`, `/camera/camera_info` | Gazebo → ROS (`camera:=true`) |

LiDAR 프레임은 로봇 정면 기준 180° 돌아 있습니다.

## 차선 시험 트랙

차선 주행 알고리즘을 시험하는 월드입니다(경기장이 아님). 평평한 바닥에 테이프 선만 있는 2차선 타원이며, 치수는 경기장 도면(`docs/arena_map_2000x2000.jpg`)과 같습니다: 직선 0.6 m, 차선 중심 반경 0.405 m·0.595 m, 차선 폭 190 mm, 테이프 20 mm. 원점이 트랙 중심이고 반시계 방향으로 달립니다.

```bash
ros2 launch pinky_parking_bringup sim.launch.py nav:=false \
  robots:=$(ros2 pkg prefix pinky_parking_bringup)/share/pinky_parking_bringup/config/lane_test.yaml
ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_follow
ros2 run pinky_parking_gazebo track_check pinky1 --laps 3      # 다른 터미널에서 채점
```

| 실행 파일 | 설명 |
|---|---|
| `track_check NAME [--laps N] [--lane K] [--change C] [--timeout S]` | Gazebo의 실제 위치로 바퀴 수, 중심선에서 벗어난 평균·최대 거리, 차체(폭 130 mm)가 테이프에 닿은 시간 비율, 시뮬레이션 속도를 출력합니다. `--change C`를 주면 차선 변경을 허용하고, 변경마다 시작·끝 위치(직선 안인지)와 넘어간 거리를 출력합니다. 변경 구간은 평균·최대에서 뺍니다. N바퀴를 돌고(변경이 C번이고 모두 직선 안이면) 종료 코드 0. `ROS_DOMAIN_ID`와 무관합니다 |
| `make_lane_world [--track FILE]` | 치수 파일로 월드 SDF를 출력합니다 |

치수는 `config/lane_track.yaml`에 있습니다. 바꾸면 월드 파일을 다시 만들고 빌드합니다(`colcon test`가 월드 파일과 치수가 맞는지 검사합니다).

```bash
ros2 run pinky_parking_gazebo make_lane_world > src/pinky_parking/pinky_parking_gazebo/worlds/lane_test.sdf
```

## 월드 추가

`worlds/`에 `.sdf`(또는 `.world`) 파일을 두고 다시 빌드한 뒤, `world_name:=<파일 이름>`으로 실행합니다.
