"""
달리고 있는 lane_follow 에 차선 변경을 요청하고 끝날 때까지 기다린다.

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_change 2
  ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_change 1 --timeout 90

lane_follow 는 --lane 으로 지금 차선을 알고 있어야 한다. 변경은 다음 직선이 시작될 때
시작한다. 끝나면 종료 코드 0, 시간 초과 2, 거절·실패 3.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import Int8
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description='차선 변경 요청')
    parser.add_argument('lane', type=int, help='목표 차선 번호')
    parser.add_argument('--timeout', type=float, default=90.0, help='제한 시간 [s, 실제 시간]')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('lane_change')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    received = []
    node.create_subscription(String, 'lane_change/status', lambda m: received.append(m.data),
                             latched)
    request = node.create_publisher(Int8, 'lane_change/request', 10)

    # 전에 남아 있던 상태(이전 요청의 결과)는 버린다
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    received.clear()

    mine = (f'waiting:{args.lane}', f'changing:{args.lane}', f'done:{args.lane}',
            f'failed:{args.lane}:')
    start, answered, last_sent, code = time.time(), False, 0.0, 2
    try:
        while time.time() - start < args.timeout:
            if not answered and time.time() - last_sent > 0.5:
                request.publish(Int8(data=args.lane))   # 연결 전에 보낸 요청이 사라질 수 있어 다시 보냄
                last_sent = time.time()
            rclpy.spin_once(node, timeout_sec=0.1)
            while received:
                status = received.pop(0)
                if not status.startswith(mine):
                    continue
                answered = True
                print(status)
                if status.startswith('done:'):
                    code = 0
                elif status.startswith('failed:'):
                    code = 3
                if code != 2:
                    raise StopIteration
        print(f'시간 초과 ({args.timeout:.0f}s). lane_follow --lane 이 실행 중인지 확인하세요')
    except (StopIteration, KeyboardInterrupt):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
