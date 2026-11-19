# pinky_parking_driving

도로 주행입니다. 카메라로 바닥의 차선 테이프를 보고 따라가는 `lane_follow`, 달리는 중에 차선을 바꾸게 하는 `lane_change`, Nav2로 경유점을 지나는 `lap`이 있습니다.

## 실행 파일

### `lane_follow`

`/camera/image_raw`에서 차선을 찾아 `/cmd_vel`을 냅니다. Nav2와 지도를 쓰지 않습니다.

```bash
ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_follow
ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_follow --speed 0.08 --timeout 120
ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_follow --lane 1     # 차선 변경 요청을 받음
```

| 인자 | 설명 |
|---|---|
| `--config FILE` | 설정 파일, 기본 `config/lane.yaml` |
| `--speed V` | 직선 속도 [m/s], 설정 파일 값을 덮어씁니다 |
| `--timeout S` | 이 시간 뒤 정지, 기본 0(계속) |
| `--lane N` | 지금 차선 번호. 주면 차선 변경 요청을 받습니다 |

| 종료 코드 | 뜻 |
|---|---|
| 0 | `--timeout` 또는 신호(Ctrl-C, SIGTERM, SIGHUP)로 멈춤 |
| 2 | 차선을 계속 못 찾아 멈춤 |
| 3 | 차선 변경이 끝나지 않아 멈춤 |

어떻게 끝나든 멈춤 명령(`/cmd_vel` 0)을 보내고 끝냅니다. 바퀴 제어는 마지막 속도 명령을 유지하기 때문입니다. 다만 `kill -9`나 전원·통신 끊김은 막지 못합니다. 스크립트에서 `ros2 run`으로 띄워 끌 때는 `ros2 run` 프로세스 하나가 아니라 프로세스 그룹 전체에 신호를 보냅니다(`setsid`로 띄운 뒤 `kill -INT -- -<PID>`). 백그라운드의 `ros2 run`은 SIGINT를 넘겨주지 않습니다.

방식: 어두운 픽셀(테이프)을 카메라 높이·기울기·화각으로 바닥 좌표로 바꾸고, 앞쪽 띠마다 차선 중심을 찾아 `lookahead` 거리의 중심점으로 향합니다(pure pursuit). 테이프가 한쪽만 보이면 그 테이프와 수직으로 차선 폭의 절반만큼 옮긴 점을 중심으로 씁니다. 계산은 `lane.py`에 있고 ROS와 분리되어 있습니다(`test/test_lane.py`).

시험 트랙과 채점은 `pinky_parking_gazebo` README의 "차선 시험 트랙"을 봅니다.

### `lane_change`

`lane_follow --lane N`으로 달리는 로봇에 차선 변경을 요청하고, 끝날 때까지 기다립니다.

```bash
ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_change 2
ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_change 1 --timeout 90
```

인자는 목표 차선 번호이고 `--timeout S`(기본 90초)를 줄 수 있습니다. 변경이 끝나면 종료 코드 0, 시간 초과 2, 거절·실패 3입니다.

- **어디서 바꾸나**: 직선 구간이 짧아(0.6 m) 변경에 필요한 약 0.5 m를 확보하려고, 곡선이 끝나고 직선이 시작되는 곳에서만 시작합니다. 직선 중간에 요청하면 다음 직선까지 기다립니다.
- **어떻게 바꾸나**: 차선을 찾는 기준을 옆 차선(차선 폭 0.19 m 옆)으로 옮기고, 조향은 평소 차선 추종 그대로 둡니다. 따로 경로를 만들지 않습니다. 따로 만든 경로는 더 길어지거나 로봇이 크게 틀어져 테이프가 카메라 밖으로 나갑니다.
- **언제 끝나나**: 새 차선 중심선까지 수직 거리가 15 mm 안이 되면 끝납니다. 0.7 m를 가도 안 끝나면, 새 차선 안(5 cm 안)이면 평소 추종으로 넘기고 아니면 멈춥니다.
- 차선 번호는 반시계 방향 주행 기준으로 안쪽이 1번입니다. 번호가 클수록 오른쪽입니다. 옆 차선으로만 바꿀 수 있습니다.
- **알려진 한계**: 시험 트랙에서 바깥으로 바꿀 때(1→2)는 새 차선 중심을 1~2 mm만 넘어가지만, 안쪽으로 바꿀 때(2→1)는 21~24 mm 넘어갑니다. 차체 여유가 20 mm라 그 순간 테이프에 몇 mm 닿습니다. 직선을 0.8 m로 늘려도 같아서 직선 길이 문제는 아닙니다. 왼쪽 곡선이 끝나는 곳에서 시작할 때 남은 회전과 겹치는 것으로 보이며, 아직 고치지 않았습니다.

토픽(로봇의 도메인 안, 대시보드 등에서 직접 써도 됩니다):

| 토픽 | 형식 | 내용 |
|---|---|---|
| `lane_change/request` | `std_msgs/Int8` | 목표 차선 번호. 같은 요청을 여러 번 보내도 한 번으로 처리합니다 |
| `lane_change/status` | `std_msgs/String` | `idle:<차선>`, `waiting:<목표>`, `changing:<목표>`, `done:<목표>`, `failed:<목표>:<이유>`. 마지막 상태를 남겨 둡니다(transient local) |

실패 이유: `no_lane`(`--lane` 없음), `invalid`, `not_adjacent`, `busy`(다른 변경 중), `no_straight`(직선 시작을 못 만남), `too_long`, `lost`(차선을 잃음).

### `lap`

```bash
ROS_DOMAIN_ID=12 ros2 run pinky_parking_driving lap
ROS_DOMAIN_ID=12 ros2 run pinky_parking_driving lap --route lap
```

| 인자 | 설명 |
|---|---|
| `--route NAME` | 경로 이름, 기본 `lap` (`config/route.yaml`) |
| `--routes FILE` | 경로 설정 파일 |
| `--timeout S` | 제한 시간, 기본 300초 |

모든 경유점을 지나면 종료 코드 0입니다.

## 설정

`config/lane.yaml` (`lane_follow`):

| 항목 | 설명 |
|---|---|
| `change` | 차선 변경: 차선 수, 직선 판단 곡률·프레임 수, 시작 허용 거리, 끝 조건, 거리 제한 [m]. 항목 설명은 파일의 주석을 봅니다 |
| `camera` | 카메라 높이·앞 거리 [m], 아래 기울기·수평 화각 [deg]. 실물은 재서 넣습니다 |
| `detect` | 영상 축소 비율, 테이프로 볼 밝기 임계값(0~255), 쓸 바닥 거리 [m] |
| `lane` | 차선 폭(테이프 중심 사이) [m], 차선을 찾는 앞 거리 범위와 띠 간격 [m] |
| `control` | 직선 속도 [m/s], 최대 회전 속도 [rad/s], 곡선 감속 계수, 정지까지 차선을 놓친 프레임 수 |

`config/route.yaml` (`lap`): 경로 이름별 경유점 목록. 각 점은 `x`, `y` [m], `yaw` [rad] (`map` 좌표). 지금 값은 공장 월드의 임시 경로입니다.
