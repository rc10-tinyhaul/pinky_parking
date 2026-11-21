# 실물 로봇 운용

실물 Pinky Pro 두 대를 이 저장소로 움직이는 절차입니다. **[현장 확인]** 표시는 실물로 아직 확인하지 않았거나 로봇 안에만 있어 저장소에서 확인할 수 없는 내용입니다. 확인하면 이 문서를 고칩니다.

## 안전

- 실물을 움직이는 명령은 사람이 로봇 옆에서 확인한 뒤 실행합니다.
- 로봇에서 속도 명령 감시 노드(`cmd_vel_watchdog`)가 항상 떠 있어야 합니다. 공식 구동 노드는 마지막 속도 명령을 계속 유지해서, PC 프로그램이 죽거나 와이파이가 끊기면 로봇이 계속 달립니다(최대 약 0.28 m/s). 연결 점검(`real_check`)이 감시 노드가 없으면 ✗로 알려 줍니다.
- 처음 켜 보는 로봇은 낮은 속도(`max_speed:=0.1`)와 짧은 동작부터 봅니다.
- 관제 화면의 **전체 정지(Esc)**로 두 대를 모두 세웁니다. 그래도 안 서면 로봇을 들어 올리거나 전원을 끕니다. **[현장 확인]** 전원 스위치 위치.
- 실물 모드(`PINKY_NETWORK=host`)에서는 시뮬레이션을 띄우지 않습니다. 같은 `ROS_DOMAIN_ID`의 토픽이 섞입니다.
- 배터리가 7 V 아래면 충전합니다. 관제 화면의 유닛 카드에 전압과 충전 %가 나옵니다. 로봇은 6.9 V에서 경고음을 냅니다.

## 구성

| 어디서 | 무엇 | 켜는 방법 |
|---|---|---|
| 로봇 | 공식 구동·라이다(`pinky_bringup`), 속도 명령 감시 노드, 배터리 | `~/pinky_start.sh` (로봇 안에 있음) |
| 로봇 | 카메라 영상과 ArUco 마커 검출 웹 서버(포트 8090) | `scripts/pinky_camera.py`를 로봇에 복사해 실행 |
| PC (Docker, 호스트 네트워크) | 시계 보정 중계(`clock_bridge`), 로봇별 Nav2(`real.launch.py`), 마커 위치 보정, 관제 서버 | `scripts/real_up.sh` |

- 로봇마다 `ROS_DOMAIN_ID`를 나누는 구조는 시뮬레이션과 같습니다(pinky1 = 11, pinky2 = 12).
- **로봇은 도메인 + 100에서 켭니다**(pinky1 = 111, pinky2 = 112). 로봇 시계가 PC와 틀려도 되도록, PC의 `clock_bridge`가 로봇 토픽의 시각을 PC 시계로 고쳐 원래 도메인(11, 12)으로 넘깁니다. 속도 명령은 반대 방향으로 넘깁니다.
- 위치는 바퀴 기록 + 라이다로 본 교실 고정벽 지도(AMCL) + 카메라로 본 ArUco 마커로 잡습니다. 실제 경기장은 바닥 테이프뿐이라 라이다에 보이지 않기 때문입니다.

## 로봇 표

| 로봇 | 도메인 (PC · 로봇) | 시작 자리 | 차종 |
|---|---|---|---|
| A = pinky1 | 11 · 111 | 11번 칸, 위(+y)를 봄. 위로 출차 | `T00` |
| B = pinky2 | 12 · 112 | 10번 칸, 아래(−y)를 봄. 아래로 출차 | `T00` (트레일러를 달면 `T10`) |

로봇 IP는 공유기가 나눠 주는 값이라 다시 켜면 바뀝니다. 아래 "로봇 IP 찾기"를 봅니다.

## 1. 네트워크

- 두 로봇과 PC를 **같은 와이파이**에 연결합니다. 로봇의 와이파이 설정은 공식 [Pinky Studio](https://github.com/pinklab-art/pinky_desktop/releases) 앱이나 로봇 터미널에서 합니다.
- 로봇은 기본으로 자체 와이파이(핫스팟)도 엽니다. 공용 와이파이만 쓰려면 로봇에서 `hostapd`만 끕니다. 핫스팟 서비스(`pinky-ap`) 전체를 끄면 로봇 웹 터미널(Jupyter)도 함께 꺼집니다.
- 공용 와이파이는 멀티캐스트를 막아 ROS 2가 서로를 자동으로 찾지 못하는 경우가 많습니다. 그래서 상대 IP를 `ROS_STATIC_PEERS`로 적어 줍니다. PC 쪽은 `real_up.sh`가 로봇 IP로 넣고, 로봇 쪽은 `pinky_start.sh`의 두 번째 인자(PC IP)로 넣습니다.

**로봇 IP 찾기.** 로봇 화면이나 Pinky Studio에서 보거나, PC에서 와이파이 MAC 주소로 찾습니다. 로봇 이름 끝 네 자리(`pinky_62b2`의 `62b2`)는 와이파이 MAC 끝 두 바이트보다 1 작습니다(`…:62:b3`).

```bash
# 서브넷에 맞게 고칩니다 (예: 192.168.204.0/22 → 204 ~ 207)
for a in 204 205 206 207; do for b in $(seq 1 254); do ping -c1 -W1 192.168.$a.$b >/dev/null & done; done; wait
ip neigh | grep -i <로봇 와이파이 MAC>
```

## 2. 로봇에서 켜기

로봇마다 브라우저로 `http://<로봇 IP>:8888` → New → Terminal을 열고 실행합니다(ssh `pinky@<로봇 IP>`도 됩니다).

```bash
bash ~/pinky_start.sh <로봇 도메인> <PC IP>      # A: 111, B: 112
setsid nohup python3 ~/pinky_camera.py --flip > ~/camera.log 2>&1 &
```

- PC가 바뀌면 `pinky_start.sh`의 PC IP를 새 PC의 IP로 바꿉니다.
- `pinky_camera.py`는 저장소의 `scripts/pinky_camera.py`를 로봇에 복사해 둔 것입니다. 카메라는 한 프로그램만 쓸 수 있어 Pinky Studio의 카메라와 함께 쓰지 못합니다. 영상은 `http://<로봇 IP>:8090/stream`에서 볼 수 있습니다.
- **[현장 확인]** `pinky_start.sh`는 로봇에만 있고 저장소에 없습니다. 연결 점검 항목으로 보아 로봇 도메인과 `ROS_STATIC_PEERS` 설정, 공식 bringup, 속도 명령 감시 노드, 배터리 발행을 하는 것으로 보입니다. 로봇에서 파일을 가져와 `scripts/`에 넣고 이 항목을 고칩니다. 그 전까지 새 로봇을 준비할 때는 위 기능을 같은 도메인에서 띄워야 하고, 감시 노드는 `pinky_parking_bringup/pinky_parking_bringup/cmd_vel_watchdog.py`를 복사해 `python3`로 실행합니다.

## 3. 로봇 놓기

시연은 주차된 상태에서 시작합니다(`pinky_parking_bringup/config/robots.yaml`).

| 로봇 | 시작 자리 | 도면 좌표 (x, y, 방향) |
|---|---|---|
| A = pinky1 | 11번 칸 | (0, +0.15), 위(+y)를 봄 |
| B = pinky2 | 10번 칸 | (0, −0.20), 아래(−y)를 봄 |

- 두 로봇을 **등을 맞대고** 놓습니다. 위치는 바퀴 축 가운데 기준으로 A가 원점(트랙 중심)에서 위로 15 cm, B가 원점에서 아래로 20 cm입니다. B는 뒤(위쪽)에 다는 트레일러가 칸 안에 들도록 칸 중심보다 5 cm 아래에 섭니다.
- A가 보는 방향이 도면의 위(+y), A의 오른쪽이 +x입니다. 원점에서 **왼쪽 1.1 m, 오른쪽 0.8 m, 앞뒤 1 m**를 비워 둡니다(2번 트랙 바깥, 왼쪽 주유구역까지).
- 교실 고정벽 지도(아래 6번)를 찍었을 때와 같은 자리에 경기장 판이 있어야 위치가 맞습니다.

## 4. PC에서 한 번에 켜기

저장소 폴더(호스트)에서 실행합니다.

```bash
scripts/real_up.sh pinky1=<A IP> pinky2=<B IP>     # 한 대만: scripts/real_up.sh pinky1=<A IP>
```

`real_up.sh`는 차례로 아래를 하고, 단계마다 실패하면 멈춥니다.

1. 로봇이 와이파이에 있는지(ping)
2. 실물 모드 컨테이너(`PINKY_NETWORK=host`)로 바꾸고 빌드
3. 연결 점검(`real_check --bridged`, 로봇을 움직이지 않음): 바퀴 기록, 라이다, 정지 감시 노드, 시계 차이, 배터리
4. 로봇별 Nav2(`real.launch.py localization:=amcl map:=maps/room.yaml`, 카메라 마커 보정 포함)
5. 관제 서버. 로그는 `logs/run/nav.log`, `logs/run/control.log`입니다.
6. Nav2와 위치가 준비될 때까지 기다림(최대 90초)

환경변수로 바꿀 수 있는 것: `MAP`(고정벽 지도, 기본 `maps/room.yaml`), `REFERENCE_DIR`(기준 경로 CSV `coords_robot_a.csv`·`coords_robot_b.csv`를 둔 폴더, 기본 `logs/reference`).

그다음 브라우저로 `http://localhost:8080`을 엽니다.

- 로봇을 시작 칸에 놓고 **주차 자리로**를 누르면 위치가 시작 자리로 맞춰집니다. 한 대만 맞추려면 그 로봇을 고르고 칸을 골라 **위치 맞추기**를 누릅니다.
- 첫 명령은 시나리오의 **[다음 단계]** 한 번처럼 짧은 것부터 합니다. 한 대만 연결했으면 다른 로봇의 행동은 건너뜁니다.
- 끄기: `scripts/real_down.sh`(Nav2와 관제 서버). 그다음 로봇 쪽 프로그램을 끕니다.

## 5. 손으로 켜기 (`real_up.sh` 없이)

설정을 바꿔 가며 시험할 때 씁니다. 실물 모드 컨테이너(`PINKY_NETWORK=host ./run.sh`) 안에서 실행합니다.

```bash
export ROS_STATIC_PEERS="<A IP>;<B IP>"
ROS_DOMAIN_ID=111 ros2 run pinky_parking_bringup real_check --bridged   # 움직이지 않음. ✗ 가 없어야 다음으로
ros2 launch pinky_parking_bringup real.launch.py only:=pinky1 localization:=amcl \
  map:=$PWD/install/pinky_parking_navigation/share/pinky_parking_navigation/maps/room.yaml \
  cameras:=pinky1=http://<A IP>:8090 max_speed:=0.1
# 새 터미널 (같은 컨테이너)
ros2 launch pinky_parking_control control.launch.py only:=pinky1 cameras:=pinky1=http://<A IP>:8090/stream
```

`real.launch.py` 인자의 기본값은 아래와 같습니다. `real_up.sh`는 `localization:=amcl`, `map:=maps/room.yaml`로 띄웁니다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `localization` | `slam` | `amcl`(고정벽 지도 + 라이다), `slam`(달리면서 맞춤), `odom`(시작 자리 + 바퀴 기록만) |
| `map` | `maps/arena_open.yaml` | 지도. `amcl`이면 고정벽 지도를 줍니다 |
| `max_speed` | `0.2` | 직진 속도 상한 [m/s]. 처음은 `0.1` |
| `goal_tolerance` | `0.08` | 도착 허용 거리 [m]. 주차·주유 진입은 관제가 3 cm로 좁힙니다 |
| `max_turn` | `0.8` | 회전 속도 상한 [rad/s] |
| `lookahead` | `0.2` | 경로 추종 주시 거리 [m] |
| `cameras` | 없음 | `이름=http://<IP>:8090`. 주면 마커로 위치를 바로잡습니다 |
| `bridge` | `true` | 시계 보정 중계. `false`면 로봇을 같은 도메인(11)에서 켜고 시계가 맞아야 합니다 |

## 6. 교실 고정벽 지도와 마커

- **고정벽 지도**(`pinky_parking_navigation/maps/room.yaml`): 로봇을 위치를 아는 자리(11번 칸)에 세우고 제자리에서 라이다로 교실 벽과 가구를 찍은 지도입니다(`snapshot_map`). 좌표가 경기장 도면 좌표와 같습니다. **장소가 바뀌거나 경기장 판을 옮기면 다시 찍습니다.** 찍을 때 사람은 비켜 섭니다.

  ```bash
  ROS_DOMAIN_ID=11 ros2 run pinky_parking_navigation snapshot_map --out <폴더>/room --x 0 --y 0.15 --yaw 1.5708
  ```

- **마커**(`pinky_parking_navigation/config/markers.yaml`): 카메라에 보이는 ArUco 마커의 지도 위치입니다. 로봇을 칸에 놓고 마커가 보일 때 관제 화면의 **마커 등록**으로 채웁니다. 지금은 교실의 0번 하나만 있습니다. 인쇄용 마커는 `scripts/make_markers.py`로 만듭니다.

## 아직 확인하지 않은 것

| 항목 | 왜 필요한가 |
|---|---|
| `~/pinky_start.sh` 내용 | 저장소에 없어 다른 로봇·장소에서 준비를 재현할 수 없고, 정지 감시처럼 안전에 걸린 설정을 검토할 수 없음 |
| 전원 스위치, 비상 정지 방법 | 안전 |
| B를 아래로 놓은 시작 자리(#24)와 시나리오 전체 | 시뮬레이션에서만 확인함 |
| 경유점 하나가 실패하면 멈추는 동작(#24) | 시뮬레이션에서만 확인함 |

## 문제 해결

| 증상 | 확인할 것 |
|---|---|
| 연결 점검에서 ✗ | 로봇에서 `pinky_start.sh`를 다시 실행합니다. 도메인(111, 112)과 PC IP 인자가 맞는지 봅니다 |
| PC에서 로봇 토픽이 안 보임 | 같은 와이파이인지, 실물 모드 컨테이너인지(`./run.sh check`), `ROS_STATIC_PEERS`에 로봇 IP가 있는지 |
| 도메인 겹침 ✗ | 같은 와이파이의 다른 팀 로봇이 같은 도메인을 씁니다. 로봇 도메인과 `robots.yaml`의 `domain_id`를 함께 바꿉니다 |
| "Nav2 가 다 켜지지 않았습니다" | 켤 때 로봇이 꺼져 있었습니다. 로봇을 켠 뒤 `real_up.sh`를 다시 실행합니다 |
| 위치가 실제와 다름 | 로봇을 시작 칸에 놓고 **주차 자리로** 또는 **위치 맞추기**. 경기장 판이 고정벽 지도를 찍었을 때와 같은 자리인지 |
| 로봇이 주행 중 자꾸 멈춤 | 감시 노드 로그("속도 명령이 없어 로봇을 세웁니다"). 와이파이가 느려 명령이 0.5 s 넘게 끊기면 감시 노드의 `--timeout`을 늘립니다 |
| 주행이 "장애물 때문에" 실패 | 라이다가 본 사람 발, 의자, 다른 로봇을 피하지 못한 것입니다. 주변을 비웁니다 |
| 시뮬레이션과 섞임 | 실물 모드에서는 시뮬레이션을 띄우지 않습니다. 시뮬레이션은 `./run.sh down` 후 `./run.sh` |
