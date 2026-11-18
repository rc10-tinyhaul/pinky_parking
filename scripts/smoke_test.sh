#!/bin/bash
# 기본 사이클 검사 (화면 없이): 두 대 띄우기 → Nav2 → pinky1 주차(park), pinky2 경로 주행(lap) → 결과
# 사용: scripts/smoke_test.sh   (ROS 환경과 이 작업공간을 source 한 셸에서. 도커 안에서도 같음)
# 성공하면 마지막 줄이 "SMOKE TEST PASSED". 환경변수: CAMERA=true|false(기본 false), GOAL_TIMEOUT(초, 기본 park 180 / lap 300), LOG_DIR
set -u
# ROS 환경이 아직 없으면 불러온다 (예: docker compose exec 로 바로 실행할 때). 이 파일 위치: <작업공간>/src/pinky_parking/scripts
if ! command -v ros2 > /dev/null; then
  WS_DIR=$(cd "$(dirname "$0")/../../.." && pwd)
  set +u   # ROS setup.bash 는 정의 안 된 변수를 참조한다
  source /opt/ros/jazzy/setup.bash
  source "$WS_DIR/install/setup.bash"
  set -u
fi
LOG=${LOG_DIR:-/tmp/pinky_smoke}; mkdir -p "$LOG"
cleanup() { kill -INT -- -"$SIM" 2>/dev/null; sleep 5; kill -9 -- -"$SIM" 2>/dev/null; }
trap cleanup EXIT
setsid ros2 launch pinky_parking_bringup sim.launch.py gui:=false headless:=true camera:=${CAMERA:-false} > "$LOG/sim.log" 2>&1 &
SIM=$!
sleep 20
echo "== 시뮬레이션 속도"
ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup sim_speed 8 || { echo "SMOKE TEST FAILED: 시뮬레이션/스캔 없음 (로그 $LOG/sim.log)"; exit 1; }
sleep 20   # Nav2 준비 (sim.launch.py 의 nav_delay 뒤에 시작)
echo "== pinky1 주차, pinky2 경로 주행 (동시에)"
ROS_DOMAIN_ID=11 ros2 run pinky_parking_docking park slot_10 --timeout ${GOAL_TIMEOUT:-180} & P1=$!
ROS_DOMAIN_ID=12 ros2 run pinky_parking_driving lap --timeout ${GOAL_TIMEOUT:-300} & P2=$!
wait $P1; R1=$?; wait $P2; R2=$?
if [ $R1 -eq 0 ] && [ $R2 -eq 0 ]; then echo "SMOKE TEST PASSED"; else echo "SMOKE TEST FAILED (park=$R1, lap=$R2, 로그 $LOG)"; exit 1; fi
