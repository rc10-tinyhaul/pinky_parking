# pinky_parking_description

Pinky Pro 차량 모델입니다. 공식 로봇 모델(`pinky_description`) 뒤에 고정형 트레일러를 붙일 수 있습니다.

## 차종 (`config/vehicles.yaml`)

| 차종 | 트레일러 | 전체 길이 (`docs/plan.md`) |
|---|---|---|
| `basic` | 없음. 공식 모델과 같습니다 | 140 mm |
| `trailer10` | 차체 뒤로 100 mm | 240 mm |
| `trailer15` | 차체 뒤로 150 mm | 290 mm |

트레일러 항목(단위 m, kg):

| 항목 | 설명 |
|---|---|
| `length` | 차체 뒤끝에서 뒤로 늘어나는 길이 |
| `width` | 폭 |
| `top` | 바닥에서 윗면까지. LiDAR 스캔 높이(약 0.125 m)보다 낮아야 합니다 |
| `clearance` | 바닥에서 아랫면까지 |
| `mass` | 무게 |

실물 트레일러를 만들면 재서 맞춥니다. 차종은 `pinky_parking_bringup`의 `robots.yaml`에서 로봇마다 고릅니다(`vehicle: trailer10`).

## 모델

`urdf/pinky_vehicle.urdf.xacro`는 공식 `robot.urdf.xacro`와 같은 인자에 `trailer_*` 인자를 더한 것입니다. 트레일러는 차체 뒤끝(`base_footprint` 기준 x = −0.075 m)에 고정된 상자(`trailer_link`)입니다.

- 바닥에서 떠 있고 바퀴가 없습니다. 무게는 차체가 받칩니다.
- 윗면이 LiDAR 스캔 높이보다 낮아서, LiDAR가 자기 트레일러를 장애물로 보지 않습니다. 실물에서 트레일러나 짐이 이보다 높으면 스캔 필터가 필요합니다.

파이썬에서는 `pinky_parking_description.vehicles`의 `vehicle_urdf(차종, namespace=…, is_sim=…)`로 URDF를, `footprint(차종 설정)`으로 Nav2 footprint를 만듭니다. footprint는 트레일러 차종만 뒤쪽을 트레일러 끝까지 늘리고, `basic`은 `None`(공식 Nav2 값)입니다.

## 곡선에서 트레일러 꼬리

트레일러가 고정형이라 곡선에서 꼬리가 바깥으로 밀립니다. 로봇이 차선 중심선을 따라 돌 때, 중심선에서 꼬리 바깥 모서리까지의 거리를 계산하면 다음과 같습니다(폭 100 mm 기준). 차선 빈 공간의 절반은 85 mm입니다.

| 차종 | 1번 차선 (곡선 반경 405 mm) | 2번 차선 (595 mm) |
|---|---|---|
| `trailer10` | 83 mm (여유 2 mm) | 73 mm |
| `trailer15` | 103 mm (테이프를 18 mm 넘음) | 88 mm (3 mm 넘음) |

도면대로 트레일러 차량은 2번 차선을 씁니다. `trailer15`는 2번 차선에서도 곡선에서 테이프에 걸칩니다.
