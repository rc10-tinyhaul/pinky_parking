# pinky_parking_bringup

시뮬레이션 전체를 띄우는 launch, 로봇 목록, 도구 명령입니다.

## `launch/sim.launch.py`

로봇 목록을 읽어 Gazebo 월드와 로봇(`pinky_parking_gazebo`)을 띄우고, `nav_delay`초 뒤 로봇별 Nav2(`pinky_parking_navigation`)를 띄웁니다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `robots` | `config/robots.yaml` | 로봇 목록 |
| `world_name` | `robots.yaml`의 `world_name` | 월드 파일 이름 |
| `map` | `pinky_parking_navigation/maps/arena.yaml` | 지도 |
| `gui` | `true` | Gazebo 창 |
| `headless` | `false` | 화면 없는 환경용 |
| `camera` | `true` | 카메라 영상 브리지. 켜면 로봇마다 시뮬레이션 카메라 서버와 마커 위치 보정(`marker_localizer`)도 뜹니다 |
| `nav` | `true` | Nav2 실행 |
| `nav_delay` | `15.0` | Nav2 시작 전 대기 [s] |

## 로봇 목록 (`config/robots.yaml`)

```yaml
world_name: arena.world
robots:
  - {name: pinky1, domain_id: 11, x: 0.0, y: 0.15, yaw: 1.5708, vehicle: T00}     # slot_11
  - {name: pinky2, domain_id: 12, x: 0.0, y: -0.20, yaw: -1.5708, vehicle: T00}   # slot_10
```

시연은 주차된 상태에서 시작합니다. A(pinky1)는 11번 칸에서 위쪽, B(pinky2)는 10번 칸에서 아래쪽을 봅니다. B는 트레일러가 칸 안에 들도록 칸 중심보다 5 cm 아래에 섭니다.

| 필드 | 설명 |
|---|---|
| `name` | 로봇 이름 |
| `domain_id` | 이 로봇의 `ROS_DOMAIN_ID` |
| `x`, `y` [m], `yaw` [rad] | 경기장 도면 좌표. 스폰 위치이자 AMCL 초기 위치 |
| `vehicle` | 차종(`pinky_parking_description`의 `config/vehicles.yaml`). 없으면 `T00` |

`config/pinky1_only.yaml`은 지도 작성용(pinky1 한 대)입니다.

## 실행 파일

`ROS_DOMAIN_ID`로 대상 로봇을 지정합니다(pinky1 = 11, pinky2 = 12).

| 명령 | 설명 |
|---|---|
| `send_goal X Y [YAW] [--timeout S]` | `map` 좌표 목표를 보내고 결과를 출력합니다. YAW [rad] 기본 0, `--timeout` 기본 120초. 성공하면 종료 코드 0 |
| `auto_explore [초]` | 지도 작성용 자동 탐색. 기본 240초 |
| `sim_speed [초]` | 시뮬레이션 속도(1.00 = 실시간)와 `/scan` 주기. 기본 8초 |
| `cmd_vel_watchdog [--timeout S]` | **실물 로봇에서 함께 띄웁니다.** 0이 아닌 속도 명령 뒤 S초(기본 0.5) 동안 새 명령이 없으면 정지 명령을 냅니다. 공식 구동 노드는 마지막 속도를 계속 유지해서, PC 프로그램이 죽거나 와이파이가 끊기면 로봇이 계속 달리기 때문입니다. 로봇에는 이 파일(`pinky_parking_bringup/cmd_vel_watchdog.py`)만 복사해 `python3`로 실행해도 됩니다. 키보드 조종은 `teleop_twist_keyboard --ros-args -p repeat_rate:=10.0`처럼 명령을 반복해서 보내야 합니다 |
