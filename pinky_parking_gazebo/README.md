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
| `camera` | `true` | 카메라 영상 브리지 |
| `cam_tilt_deg` | `8` | 카메라 기울기 [deg] |
| `vehicle` | `T00` | 차종(`pinky_parking_description`의 `config/vehicles.yaml`) |

## 토픽 (각 로봇의 도메인)

| 토픽 | 방향 |
|---|---|
| `/clock`, `/tf`(`odom → base_footprint`), `/scan`, `/odom`, `/joint_states` | Gazebo → ROS |
| `/cmd_vel` | ROS → Gazebo |
| `/camera/image_raw`, `/camera/camera_info` | Gazebo → ROS (`camera:=true`) |

LiDAR 프레임은 로봇 정면 기준 180° 돌아 있습니다.

## 월드 추가

`worlds/`에 `.sdf`(또는 `.world`) 파일을 두고 다시 빌드한 뒤, `world_name:=<파일 이름>`으로 실행합니다.
