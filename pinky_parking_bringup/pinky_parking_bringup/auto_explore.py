"""
지도 만들기용 단순 자동 탐색. 키보드 조종 대신 쓴다.

앞이 비면 직진, 막히면 넓은 쪽으로 회전, 가끔 방향을 틀어 구석도 본다.
Pinky LiDAR 프레임은 로봇 정면 기준 180도 돌아 있다 (로봇 각도 = 스캔 각도 - pi).

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup auto_explore 240     # 240초 동안
"""
import math
import random
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Explore(Node):
    def __init__(self):
        super().__init__('auto_explore')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.turn_until = 0.0
        self.turn_dir = 1.0
        self.next_kick = time.time() + 15
        self.scans = 0

    @staticmethod
    def sector(scan, lo, hi):
        values = []
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment - math.pi
            a = math.degrees(math.atan2(math.sin(a), math.cos(a)))
            if lo <= a <= hi and math.isfinite(r) and r > scan.range_min:
                values.append(r)
        return min(values) if values else 10.0

    def on_scan(self, scan):
        self.scans += 1
        now, cmd = time.time(), Twist()
        front = self.sector(scan, -30, 30)
        left, right = self.sector(scan, 30, 100), self.sector(scan, -100, -30)
        if now < self.turn_until:
            cmd.angular.z = 0.9 * self.turn_dir
        elif front < 0.35:
            self.turn_dir = 1.0 if left > right else -1.0
            self.turn_until = now + random.uniform(0.8, 2.0)
            cmd.angular.z = 0.9 * self.turn_dir
        elif now > self.next_kick:
            self.turn_dir = random.choice([-1.0, 1.0])
            self.turn_until = now + random.uniform(1.0, 3.0)
            self.next_kick = now + random.uniform(10, 20)
        else:
            cmd.linear.x = 0.18
            if min(left, right) < 0.5:
                cmd.angular.z = 0.2 * max(-1.0, min(1.0, left - right))
        self.pub.publish(cmd)


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    duration = float(args[0]) if args else 240.0
    rclpy.init()
    node = Explore()
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.pub.publish(Twist())
    print(f'탐색 끝. 받은 스캔 {node.scans}개')
    if not node.scans:
        print('/scan 이 없습니다. ROS_DOMAIN_ID 를 확인하세요.')
    return 0 if node.scans else 1


if __name__ == '__main__':
    sys.exit(main())
