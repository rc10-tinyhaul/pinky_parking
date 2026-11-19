#!/usr/bin/env bash
# 그래픽 환경을 감지해 알맞은 compose 설정으로 시뮬레이션 컨테이너를 실행한다.
#
#   ./run.sh          컨테이너를 띄우고(이미 떠 있으면 그대로) 셸로 들어감
#   ./run.sh up       컨테이너만 띄움
#   ./run.sh down     종료
#   ./run.sh check    감지 결과와 고를 설정만 출력
#
# 그래픽을 직접 고르려면: PINKY_GPU=nvidia|intel|cpu ./run.sh
# 실물 로봇과 통신하려면 호스트 네트워크로: PINKY_NETWORK=host ./run.sh   (compose.real.yaml)
set -euo pipefail
cd "$(dirname "$0")"

IMAGE=pinky_parking:jazzy
# 컨테이너 사용자를 호스트 사용자와 같은 UID/GID 로 만든다 (compose.yaml 의 build args)
USER_UID=$(id -u)
USER_GID=$(id -g)
export USER_UID USER_GID
LOCAL_DIR=.run
LOCAL_COMPOSE=$LOCAL_DIR/compose.local.yaml
NETWORK=${PINKY_NETWORK:-isolated}   # isolated: 시뮬레이션 (기본) | host: 실물 로봇

say() { echo "[run.sh] $*"; }

# ---- 감지 ----------------------------------------------------------------
gpu_vendors() {   # /sys/class/drm 의 그래픽 장치 제조사 (0x10de NVIDIA, 0x8086 Intel, 0x1002 AMD)
  cat /sys/class/drm/card[0-9]*/device/vendor 2>/dev/null | sort -u
}
has_nvidia_gpu() { gpu_vendors | grep -qx 0x10de && command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; }
has_nvidia_toolkit() { docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; }
has_other_gpu() { gpu_vendors | grep -qE '^0x(8086|1002)$'; }

# NVIDIA 를 뺀 그래픽 장치(/dev/dri 의 card*, renderD*)만 넘기는 설정을 만든다.
# Intel + NVIDIA 노트북에서 /dev/dri 전체를 넘기면 화면이 깨지기 때문이다.
write_dri_compose() {
  local nodes=() name
  for dev in /sys/class/drm/card[0-9]* /sys/class/drm/renderD*; do
    [ -e "$dev/device/vendor" ] || continue
    [ "$(cat "$dev/device/vendor")" = 0x10de ] && continue
    name=$(basename "$dev")
    [ -e "/dev/dri/$name" ] && nodes+=("/dev/dri/$name")
  done
  mkdir -p "$LOCAL_DIR"
  {
    echo "# run.sh 가 만든 파일 (NVIDIA 를 뺀 그래픽 장치)"
    echo "services:"
    echo "  sim:"
    echo "    devices:"
    for node in "${nodes[@]}"; do echo "      - $node:$node"; done
    # 컨테이너의 일반 사용자가 장치를 열 수 있게, 장치의 그룹(video, render)에 넣는다
    echo "    group_add:"
    stat -c '%g' "${nodes[@]}" | sort -u | sed 's/.*/      - "&"/'
  } > "$LOCAL_COMPOSE"
}

choose_mode() {   # nvidia | intel | cpu
  if [ -n "${PINKY_GPU:-}" ]; then echo "$PINKY_GPU"; return; fi
  if has_nvidia_gpu && has_nvidia_toolkit; then echo nvidia; return; fi
  if has_nvidia_gpu && [ -t 0 ] && [ "${1:-}" != check ]; then
    echo >&2
    echo "NVIDIA GPU가 있지만 NVIDIA Container Toolkit이 없습니다." >&2
    echo "설치하면 Gazebo·RViz가 빨라집니다. (sudo 필요, docker 재시작 → 실행 중인 컨테이너가 멈춥니다)" >&2
    read -r -p "지금 설치할까요? [y/N] " answer
    if [[ "$answer" =~ ^[yY] ]]; then
      scripts/install_nvidia_toolkit.sh >&2 && { echo nvidia; return; }
      echo "[run.sh] 설치에 실패해 다른 방법으로 실행합니다." >&2
    else
      echo "[run.sh] 나중에 설치하려면: scripts/install_nvidia_toolkit.sh" >&2
    fi
  fi
  if has_other_gpu; then echo intel; return; fi
  echo cpu
}

compose_args() {   # 모드별 compose 파일 (그래픽 + 네트워크)
  local files
  case "$1" in
    nvidia) files="-f compose.yaml -f compose.gpu.yaml" ;;
    intel)  write_dri_compose; files="-f compose.yaml -f $LOCAL_COMPOSE" ;;
    cpu)    files="-f compose.yaml" ;;
    *) say "알 수 없는 PINKY_GPU 값: $1 (nvidia|intel|cpu)"; exit 1 ;;
  esac
  case "$NETWORK" in
    isolated) echo "$files" ;;
    host)     echo "$files -f compose.real.yaml" ;;
    *) say "알 수 없는 PINKY_NETWORK 값: $NETWORK (isolated|host)"; exit 1 ;;
  esac
}

describe_network() {
  case "$1" in
    host) echo "호스트 네트워크(실물 로봇용). 시뮬레이션은 띄우지 마세요: 같은 도메인의 실물 로봇과 섞입니다." ;;
    *)    echo "격리된 네트워크(시뮬레이션용). 실물 로봇과는 통신하지 않습니다." ;;
  esac
}

describe() {
  case "$1" in
    nvidia) echo "NVIDIA GPU로 실행합니다." ;;
    intel)  echo "Intel·AMD 그래픽으로 실행합니다." ;;
    cpu)    echo "GPU 없이 CPU로 실행합니다. Gazebo·RViz가 느릴 수 있습니다(시뮬레이션은 camera:=false 권장)." ;;
  esac
}

# ---- 실행 ----------------------------------------------------------------
check_docker() {
  if ! docker info >/dev/null 2>&1; then
    say "docker 에 접속할 수 없습니다. Docker가 설치되어 있고, 사용자가 docker 그룹에 있는지 확인하세요."
    say "  sudo usermod -aG docker \$USER   (그 뒤 로그아웃 후 다시 로그인)"
    exit 1
  fi
}

running() { [ -n "$(docker compose ps -q sim 2>/dev/null)" ]; }

running_network() {   # 떠 있는 컨테이너의 네트워크: host | isolated
  local mode
  mode=$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$(docker compose ps -q sim)" 2>/dev/null)
  [ "$mode" = host ] && echo host || echo isolated
}

up() {
  check_docker
  if running; then
    if [ "$(running_network)" != "$NETWORK" ]; then
      say "컨테이너가 다른 네트워크($(running_network))로 실행 중입니다. 바꾸려면 먼저 ./run.sh down"
      exit 1
    fi
    say "이미 실행 중입니다."; return
  fi
  mode=$(choose_mode)
  say "$(describe "$mode")"
  say "$(describe_network "$NETWORK")"
  # shellcheck disable=SC2046
  docker image inspect "$IMAGE" >/dev/null 2>&1 || { say "이미지가 없어 먼저 빌드합니다 (처음 한 번, 수 분)."; docker compose build; }
  docker compose $(compose_args "$mode") up -d
}

case "${1:-shell}" in
  shell) up; docker compose exec sim bash ;;
  up)    up ;;
  down)  docker compose down ;;
  check)
    echo "그래픽 장치 제조사: $(gpu_vendors | tr '\n' ' ')"
    echo "NVIDIA GPU: $(has_nvidia_gpu && echo 있음 || echo 없음)"
    echo "NVIDIA Container Toolkit: $(has_nvidia_toolkit && echo 있음 || echo 없음)"
    echo "Intel·AMD 그래픽: $(has_other_gpu && echo 있음 || echo 없음)"
    mode=$(choose_mode check); echo "선택: $mode — $(describe "$mode")"
    echo "네트워크: $NETWORK — $(describe_network "$NETWORK")"
    running && echo "실행 중인 컨테이너의 네트워크: $(running_network)"
    echo "compose: docker compose $(compose_args "$mode")"
    ;;
  *) sed -n '2,10p' "$0"; exit 1 ;;
esac
