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
| pinky1 | 11 | `basic` | | |
| pinky2 | 12 | `trailer10` | | |

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
