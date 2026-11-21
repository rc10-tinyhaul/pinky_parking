# 실물 로봇 운용

실물 Pinky Pro를 이 저장소로 움직이는 절차입니다. 공식 [pinky_pro](https://github.com/pinklab-art/pinky_pro) README와 코드(`pinky_parking.repos`에 고정한 버전)를 기준으로 썼습니다. **[현장 확인]** 표시는 실물로 아직 확인하지 않은 내용이니, 확인하면 이 문서를 고칩니다.

## 안전

- 실물을 움직이는 명령은 사람이 확인한 뒤 실행합니다.
- 첫 시험은 낮은 속도(`max_speed:=0.1` 이하)와 짧은 목표(0.3 m 정도)로 합니다. 가능하면 바퀴가 바닥에 닿지 않게 받침 위에서 구동부터 확인합니다.
- 로봇에서 `cmd_vel_watchdog`를 항상 함께 띄웁니다. 공식 구동 노드는 마지막 속도 명령을 계속 유지해서, PC 프로그램이 죽거나 와이파이가 끊기면 로봇이 계속 달립니다(최대 약 0.28 m/s).
- 비상 시에는 로봇을 들어 올리거나 전원을 끕니다. **[현장 확인]** 전원 스위치 위치.
- 시뮬레이션과 실물을 같은 네트워크에서 동시에 쓰지 않습니다. 같은 `ROS_DOMAIN_ID`(11, 12)의 토픽이 섞입니다.

## 구성

| 어디서 | 무엇 |
|---|---|
| 로봇 (공식 패키지) | 구동·오도메트리·LiDAR·TF (`pinky_bringup bringup_robot.launch.xml`), 배터리 전압 |
| 로봇 (이 저장소에서 파일 하나) | `cmd_vel_watchdog.py` |
| PC (Docker, 호스트 네트워크) | 지도 만들기(SLAM), Nav2(`robot_nav.launch.py`), RViz, 이 저장소의 명령(`park`, `lap`, `send_goal` 등) |

로봇에는 공식 패키지만 두고, 이 저장소의 코드는 PC에서 실행합니다. 로봇마다 `ROS_DOMAIN_ID`를 나누는 구조는 시뮬레이션과 같습니다.

## 로봇 표

현장에서 채웁니다.

| 로봇 | `ROS_DOMAIN_ID` | 차종 | 와이파이 IP | 비고 |
|---|---|---|---|---|
| pinky1 | 11 | `T00` | | |
| pinky2 | 12 | `T10` | | |

## 1. 네트워크

- 공식 README의 실물 웹 주소가 `192.168.4.1`이라, 로봇은 기본으로 자체 와이파이(공유기 역할)를 여는 것으로 보입니다. **[현장 확인]**
- 한 대만 쓸 때는 PC가 그 로봇의 와이파이에 접속해도 됩니다. 두 대를 한 PC에서 쓰려면 두 로봇과 PC를 **같은 공유기**에 연결합니다. 로봇의 와이파이 설정은 공식 [Pinky Studio](https://github.com/pinklab-art/pinky_desktop/releases) 앱으로 합니다.
- ROS 2는 같은 네트워크 안에서 서로를 자동으로 찾습니다. 공유기가 멀티캐스트를 막으면(공용 와이파이에서 흔함) 찾지 못하므로, 그때는 상대 IP를 `ROS_STATIC_PEERS` 환경변수로 적어 줍니다.

## 2. 로봇 설정 (로봇에서)

**[현장 확인]** 로봇 접속 방법(SSH 사용자, 호스트 이름), 공식 이미지의 작업공간 위치, 기본 `ROS_DOMAIN_ID`.

```bash
# 로봇의 셸 설정(~/.bashrc)에 도메인을 적습니다. pinky2 는 12
export ROS_DOMAIN_ID=11
```

감시 노드는 이 저장소의 파일 하나를 PC에서 로봇으로 복사해 둡니다.

```bash
scp pinky_parking_bringup/pinky_parking_bringup/cmd_vel_watchdog.py <사용자>@<로봇 IP>:~/
```

로봇에서 두 터미널로 실행합니다.

```bash
ros2 launch pinky_bringup bringup_robot.launch.xml   # 구동, LiDAR, TF
python3 ~/cmd_vel_watchdog.py                        # 명령이 0.5 s 끊기면 정지
```

## 3. PC 연결 확인

```bash
./run.sh down                      # 시뮬레이션 컨테이너가 떠 있으면
PINKY_NETWORK=host ./run.sh        # 호스트 네트워크 컨테이너
ROS_DOMAIN_ID=11 ros2 topic list   # /scan /odom /tf /cmd_vel 이 보여야 합니다
ROS_DOMAIN_ID=11 ros2 topic hz /scan
```

- 로봇과 PC의 시계가 크게 다르면 TF 오류(`extrapolation into the future`)가 납니다. 두 쪽의 `date`를 비교하고, 차이가 크면 시간을 맞춥니다. **[현장 확인]** 로봇의 시간 동기 설정.

## 4. 지도 만들기 (PC에서)

공식 실물 지도 작성 launch를 로봇의 도메인에서 실행합니다.

```bash
ROS_DOMAIN_ID=11 ros2 launch pinky_navigation map_building.launch.xml
ROS_DOMAIN_ID=11 ros2 launch pinky_navigation map_view.launch.xml        # RViz
ROS_DOMAIN_ID=11 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0
ROS_DOMAIN_ID=11 ros2 run nav2_map_server map_saver_cli -f src/pinky_parking/pinky_parking_navigation/maps/<지도 이름>
```

- 키보드 조종은 `repeat_rate`로 명령을 반복해서 보냅니다. 감시 노드가 0.5 s 동안 명령이 없으면 로봇을 세우기 때문입니다.
- 키보드 조종의 처음 속도는 낮춰서 씁니다(`z` 키로 속도를 줄임).
- 지도는 로봇 한 대로 만듭니다.

## 5. 첫 이동 (#14 완료 조건)

```bash
ROS_DOMAIN_ID=11 ros2 launch pinky_parking_navigation robot_nav.launch.py domain_id:=11 \
  use_sim_time:=false max_speed:=0.1 map:=<지도>.yaml x:=<시작 x> y:=<시작 y> yaw:=<시작 방향>
ROS_DOMAIN_ID=11 ros2 launch pinky_navigation nav2_view.launch.xml       # RViz
ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup send_goal <x> <y> <yaw>  # 0.3 m 정도 앞
```

- `x`, `y`, `yaw`는 지도 위 로봇의 실제 시작 위치입니다. 다르면 RViz의 **2D Pose Estimate**로 맞춥니다.
- 확인할 것: 목표까지 가서 서는지, RViz의 위치가 실제와 맞는지, 감시 노드가 정상 주행 중에 로봇을 세우지 않는지.

## 5-1. 경기장 없이 좌표로만 시험 (관제센터)

경기장 출력물(테이프, 마커)이 없어 센서로 위치를 잡을 수 없을 때, **시작 위치 + 바퀴 기록(오도메트리)**으로 도면 좌표를 따라 움직여 봅니다. 지도 만들기(4번)가 필요 없습니다. 오래 달리면 바퀴 미끄러짐만큼 오차가 쌓이므로 짧은 동작부터 봅니다.

**로봇 놓기.** 시연은 주차된 상태에서 시작합니다(`pinky_parking_bringup/config/robots_arena.yaml`).

| 로봇 | 도메인 | 시작 자리 | 도면 좌표 (x, y, 방향) |
|---|---|---|---|
| A = pinky1 | 11 | 11번 칸 | (0, +0.15), 위(+y)를 봄 |
| B = pinky2 | 12 | 10번 칸 | (0, −0.20), 아래(−y)를 봄 |

- 두 로봇을 **등을 맞대고** 놓습니다. A는 위(+y), B는 아래(−y)를 봅니다. 위치는 바퀴 축 가운데 기준으로 A가 원점에서 위로 15 cm, B가 원점에서 아래로 20 cm입니다. B는 뒤(위쪽)에 다는 트레일러가 칸 안에 들도록 칸 중심보다 5 cm 아래에 섭니다. 바닥에 테이프로 표시해 두면 다시 놓기 쉽습니다.
- A가 보는 방향이 도면의 위(+y), A의 오른쪽이 +x입니다. 원점에서 **왼쪽 1.1 m, 오른쪽 0.8 m, 앞뒤 1 m**를 비워 둡니다(2번 트랙 바깥, 왼쪽 주유구역까지).
- 로봇을 켤 때(bringup을 띄울 때) 그 자리에 있어야 합니다. 옮겼으면 bringup부터 다시 켜거나, RViz의 **2D Pose Estimate**로 자리를 다시 알려 줍니다.

**한 대만 시험 (A).** B 없이 A만 연결해도 됩니다. A를 11번 칸 자리(원점에서 위로 15 cm)에 놓고, 아래 명령에 `only:=pinky1`을 붙입니다. 시나리오에서 B의 행동과 B만 있는 단계는 "B 없음"으로 건너뜁니다.

**실행 (PC, `PINKY_NETWORK=host ./run.sh` 컨테이너 안).** 로봇에서 bringup과 감시 노드가 떠 있어야 합니다(2번).

```bash
colcon build --symlink-install --packages-up-to pinky_parking_control && source install/setup.bash
ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup real_check          # 연결 점검 (움직이지 않음). "점검 통과"를 본 뒤 다음으로
ros2 launch pinky_parking_bringup real.launch.py only:=pinky1       # Nav2 (두 대면 only 를 뺌)
# 새 터미널 (같은 컨테이너)
ros2 launch pinky_parking_control control.launch.py only:=pinky1 \
  robots:=$PWD/install/pinky_parking_bringup/share/pinky_parking_bringup/config/robots_arena.yaml
# 브라우저 http://localhost:8080 → 경기장 화면. "시연 시나리오"의 [다음 단계]로 한 단계씩
```

- `real_check`는 `/odom`·`/scan`이 오는지, **같은 도메인에 다른 로봇이 섞였는지**(같은 와이파이의 다른 팀), 로봇과 PC의 시계 차이, 감시 노드, 배터리 전압을 봅니다. ✗가 있으면 고친 뒤 다시 돌립니다. 도메인이 겹치면 두 쪽 모두 다른 번호로 바꿉니다(로봇 `~/.bashrc`와 `robots_arena.yaml`).
- `real.launch.py`는 로봇마다 `robot_nav.launch.py localization:=odom use_sim_time:=false`를 띄웁니다. AMCL 대신 `odom_localizer`가 시작 자리를 도면 좌표로 보고 `map → odom`과 위치(`amcl_pose`)를 냅니다. 지도는 벽 없는 경기장 바닥(`maps/arena_open`)입니다.
- 실물용 기본값: 직진 상한 `max_speed:=0.1`, 도착 허용 거리 `goal_tolerance:=0.08`(공식 0.25 m로는 출차 0.4 m를 가다 말고 도착으로 칩니다), 회전 상한 `max_turn:=0.8`(바퀴 미끄러짐을 줄임).
- 첫 명령은 **[다음 단계] 한 번**처럼 짧은 것부터, 사람이 로봇 옆에서 보며 합니다. 화면의 "전체 정지(Esc)"와 감시 노드, 전원 스위치를 확인해 둡니다.
- 관제 화면의 유닛 카드에 배터리 전압이 나옵니다. 7 V 아래면 충전합니다.
- 라이다가 본 실제 물체(사람 발, 의자)는 장애물로 피해 갑니다. 경로가 막히면 주행이 실패하니 주변을 비웁니다.
- 같은 설정의 시뮬레이션: `sim.launch.py robots:=...robots_arena.yaml map:=.../arena_open.yaml localization:=odom max_speed:=0.1 goal_tolerance:=0.08 max_turn:=0.8`

## 6. 끝내기

PC의 명령과 Nav2를 먼저 끄고(Ctrl-C), 그다음 로봇의 bringup과 감시 노드를 끕니다. 배터리 전압은 `battery/voltage` 토픽으로 볼 수 있습니다. 공식 코드는 6.8 V 아래를 배터리 부족으로 봅니다.

## 아직 확인하지 않은 것

| 항목 | 왜 필요한가 |
|---|---|
| 로봇 와이파이 기본 모드와 IP | 네트워크 구성 |
| SSH 사용자, 작업공간 위치, 기본 `ROS_DOMAIN_ID` | 로봇 설정 |
| **카메라 드라이버와 토픽** | 공식 실물 bringup에 카메라가 없습니다. 카메라 차선 주행(`lane_follow`)에 `/camera/image_raw`가 필요합니다 |
| 카메라 높이·기울기 (자로 잼) | 차선 주행 설정(`config/lane.yaml`) |
| 시간 동기 | TF 오류 방지 |
| 전원 스위치, 비상 정지 방법 | 안전 |

## 문제 해결

| 증상 | 확인할 것 |
|---|---|
| PC에서 토픽이 안 보임 | 같은 와이파이인지, `ROS_DOMAIN_ID`가 같은지, `PINKY_NETWORK=host`로 띄웠는지, 공유기가 멀티캐스트를 막는지(`ROS_STATIC_PEERS`) |
| Nav2가 시작하지 않고 시간을 기다림 | `use_sim_time:=false`인지 |
| TF `extrapolation` 오류 | 로봇과 PC의 시계 차이 |
| 로봇이 주행 중 자꾸 멈춤 | 감시 노드 로그("속도 명령이 없어 로봇을 세웁니다"). 와이파이가 느려 명령이 0.5 s 넘게 끊기면 `--timeout`을 늘립니다 |
