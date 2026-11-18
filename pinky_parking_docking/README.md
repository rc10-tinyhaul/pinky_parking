# pinky_parking_docking

입차·출차입니다. 지금은 주차 칸 위치까지 Nav2로 이동하는 뼈대입니다.

## 실행 파일

```bash
ROS_DOMAIN_ID=11 ros2 run pinky_parking_docking park slot_10
ros2 run pinky_parking_docking park --list
```

| 인자 | 설명 |
|---|---|
| `slot` | 칸 이름 (`config/slots.yaml`) |
| `--slots FILE` | 칸 설정 파일 |
| `--timeout S` | 제한 시간, 기본 180초 |

성공하면 종료 코드 0입니다.

## 설정

`config/slots.yaml`: 칸 이름별 `x`, `y` [m], `yaw` [rad] (`map` 좌표). 지금 값은 공장 월드의 임시 위치입니다.
