# pinky_parking_driving

도로 주행입니다. 카메라로 바닥의 차선 테이프를 보고 따라가는 `lane_follow`와, Nav2로 경유점을 지나는 `lap`이 있습니다.

## 실행 파일

### `lane_follow`

`/camera/image_raw`에서 차선을 찾아 `/cmd_vel`을 냅니다. Nav2와 지도를 쓰지 않습니다.

```bash
ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_follow
ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_follow --speed 0.08 --timeout 120
```

| 인자 | 설명 |
|---|---|
| `--config FILE` | 설정 파일, 기본 `config/lane.yaml` |
| `--speed V` | 직선 속도 [m/s], 설정 파일 값을 덮어씁니다 |
| `--timeout S` | 이 시간 뒤 정지, 기본 0(계속) |

차선을 계속 못 찾으면 멈추고 종료 코드 2입니다. `--timeout`이나 Ctrl-C로 멈추면 0입니다.

방식: 어두운 픽셀(테이프)을 카메라 높이·기울기·화각으로 바닥 좌표로 바꾸고, 앞쪽 띠마다 차선 중심을 찾아 `lookahead` 거리의 중심점으로 향합니다(pure pursuit). 테이프가 한쪽만 보이면 그 테이프와 수직으로 차선 폭의 절반만큼 옮긴 점을 중심으로 씁니다. 계산은 `lane.py`에 있고 ROS와 분리되어 있습니다(`test/test_lane.py`).

시험 트랙과 채점은 `pinky_parking_gazebo` README의 "차선 시험 트랙"을 봅니다.

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
| `camera` | 카메라 높이·앞 거리 [m], 아래 기울기·수평 화각 [deg]. 실물은 재서 넣습니다 |
| `detect` | 영상 축소 비율, 테이프로 볼 밝기 임계값(0~255), 쓸 바닥 거리 [m] |
| `lane` | 차선 폭(테이프 중심 사이) [m], 차선을 찾는 앞 거리 범위와 띠 간격 [m] |
| `control` | 직선 속도 [m/s], 최대 회전 속도 [rad/s], 곡선 감속 계수, 정지까지 차선을 놓친 프레임 수 |

`config/route.yaml` (`lap`): 경로 이름별 경유점 목록. 각 점은 `x`, `y` [m], `yaw` [rad] (`map` 좌표). 지금 값은 공장 월드의 임시 경로입니다.
