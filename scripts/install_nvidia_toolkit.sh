#!/usr/bin/env bash
# NVIDIA Container Toolkit 설치 (Ubuntu·Debian). 도커 컨테이너가 NVIDIA GPU를 쓸 수 있게 한다.
# sudo 가 필요하고, 마지막에 docker 를 재시작한다(실행 중인 컨테이너가 멈춘다).
# 공식 문서: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
set -euo pipefail

say() { echo "[install_nvidia_toolkit] $*"; }

if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
  say "이미 설치되어 있습니다."; exit 0
fi
if ! grep -qiE '^(ID|ID_LIKE)=.*(ubuntu|debian)' /etc/os-release; then
  say "Ubuntu·Debian이 아닙니다. 공식 문서를 따라 설치하세요."; exit 1
fi
if ! nvidia-smi >/dev/null 2>&1; then
  say "NVIDIA 드라이버가 동작하지 않습니다(nvidia-smi 실패). 드라이버를 먼저 설치하세요."; exit 1
fi

KEYRING=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor --yes -o "$KEYRING"
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed "s#deb https://#deb [signed-by=$KEYRING] https://#g" \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

if docker info --format '{{json .Runtimes}}' | grep -q '"nvidia"'; then
  say "설치했습니다."
else
  say "설치는 끝났지만 docker 에 nvidia 런타임이 보이지 않습니다. 공식 문서를 확인하세요."; exit 1
fi
