# pinky_parking_driving

도로 주행입니다. 지금은 경유점을 차례로 지나는 뼈대입니다.

## 실행 파일

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

`config/route.yaml`: 경로 이름별 경유점 목록. 각 점은 `x`, `y` [m], `yaw` [rad] (`map` 좌표). `lap`은 1번 트랙을 반시계로 도는 경유점입니다. 차선 중심선을 따라가는 주행은 관제센터(`pinky_parking_control`)의 `drive`가 합니다.
