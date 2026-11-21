# pinky_parking_navigation

로봇별 Nav2와 지도입니다.

## `launch/robot_nav.launch.py`

로봇 한 대의 Nav2(`pinky_navigation/launch/bringup_launch.xml`)를 `domain_id` 도메인에서 실행합니다. 시뮬레이션과 실물에 같이 씁니다(공식 실물용과 시뮬레이션용 launch는 기본값만 다릅니다).

| 인자 | 기본값 | 설명 |
|---|---|---|
| `domain_id` | (필수) | 이 로봇의 `ROS_DOMAIN_ID` |
| `x`, `y`, `yaw` | `0.0` | AMCL 초기 위치 [m], 방향 [rad] |
| `vehicle` | `T00` | 차종(`pinky_parking_description`). 트레일러 차종이면 footprint를 트레일러 끝까지 늘립니다 |
| `map` | `maps/arena.yaml` | 지도 YAML. 경기장 지도는 `scripts/make_arena_sim.py`가 만듭니다(실물용 벽 없는 지도는 `arena_open.yaml`) |
| `params_file` | `pinky_navigation/params/nav2_params.yaml` | 기준 Nav2 파라미터 |
| `use_sim_time` | `true` | 시뮬레이션 시계(`/clock`). **실물 로봇은 `false`** |
| `max_speed` | (공식 값 0.25) | 직진 속도 상한 [m/s]. 컨트롤러 목표 속도와 속도 평활기 최대 속도를 함께 낮춥니다. 실물 첫 시험은 `0.1` 이하로 |

AMCL 초기 위치, `lifecycle_nodes_nav`(`smoother_server` 추가), 트레일러 차종의 local·global costmap footprint는 덮어씁니다(`params.py`). `T00`은 공식 footprint(±0.06 m 정사각형)를 그대로 씁니다. 목표 도착 허용 오차는 0.25 m입니다(`xy_goal_tolerance`).

## 마커 위치 보정 (`marker_localizer`)

`robot_nav.launch.py`에 `camera_url`을 주면 함께 뜹니다. 실물은 로봇의 카메라 서버(`http://<로봇 IP>:8090`), 시뮬레이션은 `sim.launch.py`가 카메라를 켤 때 시뮬레이션 카메라 서버(`http://localhost:8011`, pinky2는 8012)를 넘깁니다. 카메라 서버의 `/markers`를 읽어, 등록된 마커(`config/markers.yaml`)가 보이면 로봇 위치를 바로잡습니다(`initialpose`).

| 토픽 | 방향 | 내용 |
|---|---|---|
| `marker_detections` | 냄 | 지금(0.5초 안) 보이는 마커마다 `id`, 로봇 기준 `x`(앞)·`y`(왼쪽) [m], `yaw`(마커 오른쪽 변의 방향) [rad], `distance` [m], `bearing`(보이는 방향, + = 왼쪽) [rad], `registered` (JSON 문자열) |
| `marker_status` | 냄 | 보이는 마커, 등록된 마커, 마지막 보정, 마커로 계산한 위치와 지금 위치의 차이 (관제 화면용) |
| `marker_register` | 받음 | 로봇의 진짜 위치(JSON `{x, y, yaw}`)를 받아 지금 보이는 마커의 지도 위치를 등록 |

- 카메라 값(초점 거리, 화면 중심, 앞쪽 거리)은 카메라 서버 응답에 있으면(시뮬레이션) 그 값을, 없으면 `markers.yaml`의 `camera`(실물 카메라)를 씁니다.
- `markers.yaml`의 10·11·20·30번은 경기장 도면 값입니다. 실물 경기장에 붙인 뒤 "마커 등록"을 하면 실제 자리로 덮어씁니다.

## 지도 만들기

경기장 지도는 `scripts/make_arena_sim.py`가 월드와 함께 만듭니다. 다른 월드의 지도를 SLAM으로 만들 때는 한 대만 띄워 만듭니다(두 대면 다른 로봇이 벽처럼 지도에 남습니다). 작업공간 루트에서 실행합니다.

```bash
ros2 launch pinky_parking_bringup sim.launch.py nav:=false \
  robots:=$(ros2 pkg prefix pinky_parking_bringup)/share/pinky_parking_bringup/config/pinky1_only.yaml \
  world_name:=<월드 파일>          # 비우면 arena.world
ROS_DOMAIN_ID=11 ros2 launch pinky_navigation gz_map_building.launch.xml
ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup auto_explore 240
ROS_DOMAIN_ID=11 ros2 run nav2_map_server map_saver_cli -f src/pinky_parking/pinky_parking_navigation/maps/my_map
```

새 지도는 `sim.launch.py map:=<경로>/my_map.yaml`로 지정하거나, `maps/`에 두고 다시 빌드합니다.
