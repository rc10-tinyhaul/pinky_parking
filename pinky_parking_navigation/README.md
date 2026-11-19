# pinky_parking_navigation

로봇별 Nav2와 지도입니다.

## `launch/robot_nav.launch.py`

로봇 한 대의 Nav2(`pinky_navigation/launch/gz_bringup_launch.xml`)를 `domain_id` 도메인에서 실행합니다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `domain_id` | (필수) | 이 로봇의 `ROS_DOMAIN_ID` |
| `x`, `y`, `yaw` | `0.0` | AMCL 초기 위치 [m], 방향 [rad] |
| `vehicle` | `basic` | 차종(`pinky_parking_description`). 트레일러 차종이면 footprint를 트레일러 끝까지 늘립니다 |
| `map` | `maps/factory.yaml` | 지도 YAML |
| `params_file` | `pinky_navigation/params/nav2_params.yaml` | 기준 Nav2 파라미터 |

AMCL 초기 위치, `lifecycle_nodes_nav`(`smoother_server` 추가), 트레일러 차종의 local·global costmap footprint는 덮어씁니다(`params.py`). `basic`은 공식 footprint(±0.06 m 정사각형)를 그대로 씁니다. 목표 도착 허용 오차는 0.25 m입니다(`xy_goal_tolerance`).

## 지도 만들기

한 대만 띄워 만듭니다(두 대면 다른 로봇이 벽처럼 지도에 남습니다). 작업공간 루트에서 실행합니다.

```bash
ros2 launch pinky_parking_bringup sim.launch.py nav:=false \
  robots:=$(ros2 pkg prefix pinky_parking_bringup)/share/pinky_parking_bringup/config/pinky1_only.yaml
ROS_DOMAIN_ID=11 ros2 launch pinky_navigation gz_map_building.launch.xml
ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup auto_explore 240
ROS_DOMAIN_ID=11 ros2 run nav2_map_server map_saver_cli -f src/pinky_parking/pinky_parking_navigation/maps/my_map
```

새 지도는 `sim.launch.py map:=<경로>/my_map.yaml`로 지정하거나, `maps/`에 두고 다시 빌드합니다.
