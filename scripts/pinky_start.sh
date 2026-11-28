#!/bin/bash
# 로봇에서 실행: 구동(bringup)과 정지 감시를 켠다. 사용: ./pinky_start.sh <로봇 도메인> <PC IP>
# 이 와이파이는 자동 탐색(멀티캐스트)이 막혀 있어 PC 주소를 ROS_STATIC_PEERS 로 알려 준다.
#
# 로봇의 홈 폴더에 이 파일과 cmd_vel_watchdog.py
# (pinky_parking_bringup/pinky_parking_bringup/cmd_vel_watchdog.py) 를 복사해 둔다.
# 로그: ~/bringup.log, ~/watchdog.log
DOMAIN=${1:?로봇 도메인 (예: 111)}; PC=${2:?PC IP}
source /opt/ros/jazzy/setup.bash; source ~/pinky_pro/install/local_setup.bash
pkill -INT -f bringup_robot.launch.xml; pkill -f cmd_vel_watchdog.py; sleep 4
pkill -9 -f "pinky_bringup|sllidar_node|robot_state_publisher|joint_state_publisher|battery_publisher" 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
export ROS_DOMAIN_ID=$DOMAIN ROS_STATIC_PEERS=$PC
setsid nohup python3 ~/cmd_vel_watchdog.py > ~/watchdog.log 2>&1 &
setsid nohup ros2 launch pinky_bringup bringup_robot.launch.xml > ~/bringup.log 2>&1 &
sleep 10
grep -E "started successfully|ERROR|Failed|health status" ~/bringup.log | cut -c1-150
pgrep -af "cmd_vel_watchdog|bringup_robot" | cut -c1-90
