#!/bin/bash
# scripts/real_up.sh 로 켠 Nav2 와 관제 서버를 끈다 (로봇 쪽 프로그램과 컨테이너는 그대로).
cd "$(dirname "$0")/.." || exit 1
docker compose exec -T sim bash -c '
  pkill -INT -f "[r]eal.launch.py"; pkill -INT -f "[c]ontrol.launch.py"; sleep 3
  for p in "[c]omponent_container" "[c]lock_bridge" "[m]arker_localizer" "[o]dom_localizer" "[t]f_pose" \
           "[a]sync_slam" "[c]ontrol_server" "[r]eal.launch.py" "[c]ontrol.launch.py"; do pkill -9 -f "$p"; done
  exit 0' >/dev/null 2>&1
[ "${1:-}" = quiet ] || echo "Nav2 와 관제 서버를 껐습니다"
