#!/bin/bash
# 실물 로봇 여러 대를 PC 에서 한 번에 켠다: 실물 모드 컨테이너 → 연결 점검 → Nav2 → 관제 서버.
#
#   scripts/real_up.sh pinky1=192.168.204.78 pinky2=192.168.204.90
#   scripts/real_down.sh                                  # 끄기
#
# 먼저 로봇마다 (브라우저 http://<로봇 IP>:8888 → New → Terminal) 아래를 실행해 둔다:
#   bash ~/pinky_start.sh <도메인 + 100> <PC IP>          # pinky1 = 111, pinky2 = 112
#   setsid nohup python3 ~/pinky_camera.py --flip > ~/camera.log 2>&1 &
#
# 로그: logs/run/nav.log, logs/run/control.log.  관제센터: http://localhost:8080
# 환경변수: MAP (고정벽 지도 YAML, 기본 maps/room.yaml), REFERENCE_DIR (가제보 기준 경로 CSV 폴더)
set -u
cd "$(dirname "$0")/.." || exit 1
[ $# -ge 1 ] || { sed -n '2,13p' "$0"; exit 1; }

NAMES=(); IPS=()
for item in "$@"; do
  case "$item" in *=*) NAMES+=("${item%%=*}"); IPS+=("${item#*=}") ;; *) echo "이름=IP 형식이 아닙니다: $item"; exit 1 ;; esac
done
ONLY=$(IFS=,; echo "${NAMES[*]}")
PEERS=$(IFS=';'; echo "${IPS[*]}")
WS=/ws/src/pinky_parking
MAP=${MAP:-$WS/pinky_parking_navigation/maps/room.yaml}
REFERENCE_DIR=${REFERENCE_DIR:-logs/reference}
ROBOTS_YAML=pinky_parking_bringup/config/robots_arena.yaml
say() { printf '\n== %s\n' "$*"; }

say "1. 로봇이 와이파이에 있는지"
for i in "${!NAMES[@]}"; do
  if ping -c1 -W2 -q "${IPS[$i]}" >/dev/null 2>&1; then echo "  ${NAMES[$i]} ${IPS[$i]}: 응답"
  else echo "  ${NAMES[$i]} ${IPS[$i]}: 응답 없음 (전원, 와이파이, IP 확인)"; exit 1; fi
done

say "2. 실물 모드 컨테이너 (호스트 네트워크)"
MODE=$(docker inspect -f '{{.HostConfig.NetworkMode}}' pinky_parking-sim-1 2>/dev/null)
if [ "$MODE" != host ]; then
  ./run.sh down >/dev/null 2>&1
  PINKY_NETWORK=host ./run.sh up | tail -1
  docker compose exec -T sim bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to $(colcon list --base-paths src/pinky_parking --names-only) 2>&1 | tail -1'
else
  echo "  이미 실물 모드"
fi
"$(dirname "$0")/real_down.sh" quiet

say "3. 연결 점검 (움직이지 않음)"
for name in "${NAMES[@]}"; do
  domain=$(python3 -c "import yaml,sys;print([r['domain_id'] for r in yaml.safe_load(open('$ROBOTS_YAML'))['robots'] if r['name']=='$name'][0])") || { echo "  $ROBOTS_YAML 에 $name 이 없습니다"; exit 1; }
  echo "  -- $name (로봇 도메인 $((domain + 100)))"
  OUT=$(docker compose exec -T -e ROS_DOMAIN_ID=$((domain + 100)) -e ROS_STATIC_PEERS="$PEERS" sim bash -c \
    'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run pinky_parking_bringup real_check --bridged --seconds 4 2>&1 | grep -v ros2run')
  echo "$OUT" | sed 's/^/    /'
  # 구동·라이다가 안 오는 로봇에 Nav2 를 띄우면 일부만 켜진 채 멈춘다 (명령을 거절함). 여기서 멈춘다
  if echo "$OUT" | grep -q -E "✗ (오도메트리|라이다|정지 감시)"; then
    echo "  $name 이 준비되지 않았습니다. 로봇에서 pinky_start.sh 를 실행한 뒤 다시 실행하세요"; exit 1
  fi
done

CAMS=(); REFS=()
for i in "${!NAMES[@]}"; do
  CAMS+=("${NAMES[$i]}=http://${IPS[$i]}:8090")
  letter=$(echo "${NAMES[$i]}" | sed 's/pinky1/a/;s/pinky2/b/')
  [ -f "$REFERENCE_DIR/coords_robot_$letter.csv" ] && REFS+=("${NAMES[$i]}=$WS/$REFERENCE_DIR/coords_robot_$letter.csv")
done
CAMERAS=$(IFS=,; echo "${CAMS[*]}")
STREAMS=$(echo "$CAMERAS" | sed 's#:8090\(,\|$\)#:8090/stream\1#g')
REFERENCES=$(IFS=,; echo "${REFS[*]:-}")
mkdir -p logs/run

say "4. Nav2 (바퀴 기록 + 라이다 고정벽 지도 + 카메라 마커)"
docker compose exec -d -e ROS_STATIC_PEERS="$PEERS" sim bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec ros2 launch pinky_parking_bringup real.launch.py only:=$ONLY localization:=amcl map:=$MAP cameras:=$CAMERAS > $WS/logs/run/nav.log 2>&1"
echo "  로그: logs/run/nav.log"

say "5. 관제 서버"
docker compose exec -d sim bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec ros2 launch pinky_parking_control control.launch.py only:=$ONLY log_dir:=$WS/logs cameras:=$STREAMS ${REFERENCES:+references:=$REFERENCES} room_map:=$MAP robots:=/ws/install/pinky_parking_bringup/share/pinky_parking_bringup/config/robots_arena.yaml > $WS/logs/run/control.log 2>&1"
echo "  로그: logs/run/control.log"

say "6. 준비 기다리기 (최대 90초)"
for _ in $(seq 1 45); do
  STATE=$(curl -s -m 2 localhost:8080/api/robots | python3 -c "
import json, sys
d = json.load(sys.stdin)
print(' | '.join(f\"{r['name']} Nav2 {'켜짐' if r['nav2_ready'] else '대기'} 위치 {'확인' if r.get('pose_confirmed') else '미확인'} 배터리 {r.get('battery_v')}\" for r in d))
print('READY' if d and all(r['nav2_ready'] and r['localized'] for r in d) else '')" 2>/dev/null)
  case "$STATE" in *READY*) break ;; esac
  sleep 2
done
echo "  ${STATE%READY*}"
echo
echo "관제센터: http://localhost:8080   (끄기: scripts/real_down.sh)"
