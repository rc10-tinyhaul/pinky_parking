"""
주차 칸으로 이동한다 (뼈대: config/slots.yaml 의 칸 위치까지 Nav2 로 이동).

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_docking park slot_10
  ros2 run pinky_parking_docking park --list

대상 로봇은 ROS_DOMAIN_ID 로 고른다 (pinky1 = 11, pinky2 = 12). 성공하면 종료 코드 0.
"""
import argparse
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
import rclpy
import yaml


def load_slots(path=None):
    path = path or Path(get_package_share_directory('pinky_parking_docking')) / 'config/slots.yaml'
    return yaml.safe_load(Path(path).read_text())['slots']


def pose_in_map(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'   # stamp 0 = 가장 최근 TF 를 쓴다 (시뮬레이션 시간과 어긋나지 않게)
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


def main():
    parser = argparse.ArgumentParser(description='주차 칸으로 이동')
    parser.add_argument('slot', nargs='?', help='칸 이름 (예: slot_10)')
    parser.add_argument('--slots', help='칸 설정 파일 (기본: config/slots.yaml)')
    parser.add_argument('--list', action='store_true', help='칸 목록만 출력')
    parser.add_argument('--timeout', type=float, default=180.0, help='제한 시간 [s, 실제 시간]')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    slots = load_slots(args.slots)
    if args.list or not args.slot:
        for name, s in slots.items():
            print(f'{name}: x={s["x"]}, y={s["y"]}, yaw={s["yaw"]}')
        return 0 if args.list else 1
    if args.slot not in slots:
        print(f'없는 칸입니다: {args.slot} (있는 칸: {", ".join(slots)})')
        return 1

    rclpy.init()
    nav = BasicNavigator('park')
    # localizer 를 amcl 로 두면 BasicNavigator 가 초기 위치 (0, 0) 을 보내 AMCL 위치를 덮어쓸 수 있다.
    # AMCL 초기 위치는 Nav2 를 띄울 때 이미 정해지므로, Nav2 가 켜질 때까지만 기다린다.
    nav.waitUntilNav2Active(localizer='robot_localization')

    s = slots[args.slot]
    start = time.time()
    nav.goToPose(pose_in_map(s['x'], s['y'], s['yaw']))
    while not nav.isTaskComplete():
        if time.time() - start > args.timeout:
            nav.cancelTask()
            print(f'{args.slot}: 시간 초과 ({args.timeout:.0f}s)')
            return 2
        time.sleep(0.2)
    result = nav.getResult()
    print(f'{args.slot}: {result.name}, {time.time() - start:.1f}s')
    nav.destroyNode()
    rclpy.shutdown()
    return 0 if result == TaskResult.SUCCEEDED else 3


if __name__ == '__main__':
    sys.exit(main())
