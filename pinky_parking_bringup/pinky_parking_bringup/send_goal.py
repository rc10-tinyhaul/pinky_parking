"""
현재 ROS_DOMAIN_ID 의 로봇에 Nav2 목표를 보내고 결과를 출력한다.

사용 예:

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup send_goal 0.3 -1.1 0.0
  ROS_DOMAIN_ID=12 ros2 run pinky_parking_bringup send_goal -0.8 1.15 3.14 --timeout 120
"""
import argparse
import math
import os
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(
        description='Nav2 목표 하나를 보내고 결과를 기다린다 (map 좌표)')
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('yaw', type=float, nargs='?', default=0.0, help='rad')
    parser.add_argument('--timeout', type=float, default=120.0, help='제한 시간 [s, 실제 시간]')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    tag = f'[domain {os.environ.get("ROS_DOMAIN_ID", "0")}]'

    rclpy.init()
    node = Node('send_goal')
    pose = {}

    def on_pose(msg):
        pose.update(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y)

    node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', on_pose, 10)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=60):
        print(tag, 'navigate_to_pose 서버가 없습니다. 이 도메인에 Nav2 가 떠 있는지 확인하세요.')
        return 1

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x, goal.pose.pose.position.y = args.x, args.y
    goal.pose.pose.orientation.z = math.sin(args.yaw / 2)
    goal.pose.pose.orientation.w = math.cos(args.yaw / 2)
    start = time.time()
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    handle = future.result()
    if not handle or not handle.accepted:
        print(tag, '목표가 거절되었습니다.')
        return 1
    result = handle.get_result_async()
    while not result.done() and time.time() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not result.done():
        handle.cancel_goal_async()
        rclpy.spin_once(node, timeout_sec=1)
        print(tag, f'시간 초과 ({args.timeout:.0f}s). 마지막 AMCL 위치 {pose}')
        return 2
    status = result.result().status
    names = {GoalStatus.STATUS_SUCCEEDED: '성공', GoalStatus.STATUS_ABORTED: '중단',
             GoalStatus.STATUS_CANCELED: '취소'}
    elapsed = time.time() - start
    print(tag, f'목표 ({args.x}, {args.y}) → {names.get(status, status)}, {elapsed:.1f}s')
    return 0 if status == GoalStatus.STATUS_SUCCEEDED else 3


if __name__ == '__main__':
    sys.exit(main())
