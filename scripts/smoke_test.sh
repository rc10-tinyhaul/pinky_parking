#!/bin/bash
# 경기장 사이클 검사 (화면 없이): 두 대를 주차 칸에 띄우기 → Nav2 → 관제 서버 → 관제 API 로
#   1. 시연 시나리오 1·2단계 (config/scenario.yaml): 두 대 출차, A 1번 트랙 한 바퀴,
#      B 는 A 가 멀어진 뒤 2번 트랙으로 차선 변경해 한 바퀴
#   2. 두 대 제자리 주차 (A = pinky1 11번 칸, B = pinky2 10번 칸)
# 사용: scripts/smoke_test.sh   (ROS 환경과 이 작업공간을 source 한 셸에서. 도커 안에서도 같음)
# 성공하면 마지막 줄이 "SMOKE TEST PASSED". 실패하면 실패한 단계와 작업 기록을 출력한다.
# 환경변수: CAMERA=true|false(기본 false), GOAL_TIMEOUT(단계마다 초, 기본 240), LOG_DIR, PORT(기본 8090)
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
PORT=${PORT:-8090}
API=http://127.0.0.1:$PORT/api
TIMEOUT=${GOAL_TIMEOUT:-240}
SIM=; CTL=
cleanup() {
  for pid in $SIM $CTL; do kill -INT -- -"$pid" 2>/dev/null; done
  sleep 5
  for pid in $SIM $CTL; do kill -9 -- -"$pid" 2>/dev/null; done
}
trap cleanup EXIT
fail() { echo "SMOKE TEST FAILED: $* (로그 $LOG)"; exit 1; }
json() { python3 -c "import json,sys; d=json.load(sys.stdin); print(eval(sys.argv[1], {}, {'d': d}))" "$1"; }

setsid ros2 launch pinky_parking_bringup sim.launch.py gui:=false headless:=true camera:=${CAMERA:-false} > "$LOG/sim.log" 2>&1 &
SIM=$!
sleep 20
echo "== 시뮬레이션 속도"
ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup sim_speed 8 || fail "시뮬레이션/스캔 없음"

setsid ros2 run pinky_parking_control control_server --port "$PORT" > "$LOG/control.log" 2>&1 &
CTL=$!
echo "== Nav2 와 위치 준비 대기"
READY=
for _ in $(seq 1 90); do
  READY=$(curl -sf "$API/robots" | json "len(d) == 2 and all(r['nav2_ready'] and r['localized'] for r in d)" 2>/dev/null)
  [ "$READY" = "True" ] && break
  sleep 2
done
[ "$READY" = "True" ] || fail "Nav2 가 준비되지 않음"
sleep 5   # AMCL 초기 위치 안정

send() {   # send <로봇> <행동> <JSON>
  curl -sf -X POST "$API/robots/$1/$2" -H "X-Control-Password: ${PINKY_CONTROL_PASSWORD:-}" \
    -H 'Content-Type: application/json' -d "$3" > /dev/null || fail "$1 $2 요청 거절"
}
wait_done() {   # wait_done <로봇...>: 작업이 끝날 때까지 기다리고 모두 succeeded 인지 본다
  local end=$(( $(date +%s) + TIMEOUT )) states=
  while [ "$(date +%s)" -lt "$end" ]; do
    states=$(curl -sf "$API/robots" | json "' '.join((r['task'] or {}).get('state', '-') for r in d if r['name'] in '$*'.split())" 2>/dev/null)
    case "$states" in ''|*running*|*pending*) sleep 2 ;; *) break ;; esac   # 비었으면 응답을 못 받은 것: 다시
  done
  curl -sf "$API/log?limit=$#" | json "'\n'.join(f\"  {t['robot']} {t['action']} {t['target']}: {t['state']} {t['detail']} ({t['elapsed_s']}s)\" for t in d)"
  [ "$states" = "$(printf 'succeeded %.0s' "$@" | sed 's/ $//')" ] || fail "작업 상태: $states"
}

scenario_step() {   # scenario_step <번호(0부터)>: 시나리오 한 단계를 실행하고 done 인지 본다
  local end=$(( $(date +%s) + TIMEOUT )) state=
  curl -sf -X POST "$API/scenario/next" -H "X-Control-Password: ${PINKY_CONTROL_PASSWORD:-}" \
    -H 'Content-Type: application/json' -d "{\"index\": $1}" > /dev/null || fail "시나리오 $(( $1 + 1 ))단계 요청 거절"
  while [ "$(date +%s)" -lt "$end" ]; do
    state=$(curl -sf "$API/scenario" | json "'running' if d['running'] else d['steps'][$1]['state']" 2>/dev/null)
    case "$state" in ''|running) sleep 2; continue ;; esac   # 비었으면 응답을 못 받은 것: 다시
    break
  done
  curl -sf "$API/scenario" | json "'  ' + d['steps'][$1]['title'] + ': ' + d['steps'][$1]['state'] + ' ' + (d['steps'][$1]['note'] or '')"
  curl -sf "$API/log?limit=6" | json "'\n'.join(f\"    {t['robot']} {t['action']} {t['target']}: {t['state']} {t['detail']} ({t['elapsed_s']}s)\" for t in d)"
  [ "$state" = done ] || fail "시나리오 $(( $1 + 1 ))단계: $state"
}

echo "== 1. 시연 시나리오 1단계 (주차 구역에서 시작)"
scenario_step 0
echo "== 2. 시연 시나리오 2단계 (출차, 1바퀴)"
scenario_step 1
echo "== 3. 두 대 제자리 주차"
send pinky1 park '{"slot": "slot_11"}'
send pinky2 park '{"slot": "slot_10"}'
wait_done pinky1 pinky2
echo "SMOKE TEST PASSED"
