#!/bin/bash
set -e
# ROS 환경을 불러온 뒤 명령 실행
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
exec "$@"
