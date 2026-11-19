#!/bin/bash
# 관제 API 검사 (화면 없이): 두 대 띄우기 → Nav2 → 관제 서버 → API 로 pinky1 주차, pinky2 주행 → 결과
# 사용: scripts/control_smoke_test.sh   (smoke_test.sh 와 같은 환경. 도커 안에서도 같음)
# 성공하면 마지막 줄이 "CONTROL SMOKE TEST PASSED". 환경변수: CAMERA, GOAL_TIMEOUT(초, 기본 300), LOG_DIR, PORT(기본 8090)
set -u
if ! command -v ros2 > /dev/null; then
  WS_DIR=$(cd "$(dirname "$0")/../../.." && pwd)
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$WS_DIR/install/setup.bash"
  set -u
fi
LOG=${LOG_DIR:-/tmp/pinky_control_smoke}; mkdir -p "$LOG"
PORT=${PORT:-8090}
API=http://127.0.0.1:$PORT/api
cleanup() {
  kill -INT -- -"$SIM" -"$CTL" 2>/dev/null; sleep 5; kill -9 -- -"$SIM" -"$CTL" 2>/dev/null
}
trap cleanup EXIT
setsid ros2 launch pinky_parking_bringup sim.launch.py gui:=false headless:=true camera:=${CAMERA:-false} > "$LOG/sim.log" 2>&1 &
SIM=$!
setsid ros2 run pinky_parking_control control_server --port "$PORT" > "$LOG/control.log" 2>&1 &
CTL=$!

json() { python3 -c "import json,sys; d=json.load(sys.stdin); print(eval(sys.argv[1], {}, {'d': d}))" "$1"; }

echo "== Nav2 준비 대기 (관제 API 의 nav2_ready)"
for _ in $(seq 1 90); do
  READY=$(curl -sf "$API/robots" | json "all(r['nav2_ready'] for r in d) and len(d) > 0" 2>/dev/null)
  [ "$READY" = "True" ] && break
  sleep 2
done
[ "$READY" = "True" ] || { echo "CONTROL SMOKE TEST FAILED: Nav2 가 준비되지 않음 (로그 $LOG)"; exit 1; }
sleep 5   # AMCL 초기 위치 안정

echo "== API 로 pinky1 주차(slot_10), pinky2 주행(lap)"
curl -sf -X POST "$API/robots/pinky1/park" -H "X-Control-Password: ${PINKY_CONTROL_PASSWORD:-}" -H 'Content-Type: application/json' -d '{"slot": "slot_10"}' > /dev/null \
  || { echo "CONTROL SMOKE TEST FAILED: park 요청 거절"; exit 1; }
curl -sf -X POST "$API/robots/pinky2/lap" -H "X-Control-Password: ${PINKY_CONTROL_PASSWORD:-}" -H 'Content-Type: application/json' -d '{"route": "lap"}' > /dev/null \
  || { echo "CONTROL SMOKE TEST FAILED: lap 요청 거절"; exit 1; }

END=$(( $(date +%s) + ${GOAL_TIMEOUT:-300} ))
while [ "$(date +%s)" -lt "$END" ]; do
  STATES=$(curl -sf "$API/robots" | json "' '.join((r['task'] or {}).get('state', '-') for r in d)")
  case "$STATES" in *running*) sleep 2 ;; *) break ;; esac
done
echo "작업 상태: $STATES"
curl -sf "$API/log?limit=2" | json "'\n'.join(f\"{t['robot']} {t['action']} {t['target']}: {t['state']} {t['detail']} ({t['elapsed_s']}s)\" for t in d)"
if [ "$STATES" = "succeeded succeeded" ]; then echo "CONTROL SMOKE TEST PASSED"; else echo "CONTROL SMOKE TEST FAILED (로그 $LOG)"; exit 1; fi
