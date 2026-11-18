"""
시뮬레이션이 실제 시간 대비 얼마나 빨리 도는지(/clock)와 /scan 주기를 잰다.

사용 예:

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup sim_speed        # 1.00 이면 실시간
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    seconds = float(args[0]) if args else 8.0
    rclpy.init()
    node = Node('sim_speed')
    data = {'scans': 0}

    def on_clock(msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        data.setdefault('first', (time.time(), t))
        data['last'] = (time.time(), t)

    def on_scan(msg):
        data['scans'] += 1

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if 'first' not in data or data['last'][0] == data['first'][0]:
        print('/clock 이 없습니다. 시뮬레이션과 ROS_DOMAIN_ID 를 확인하세요.')
        return 1
    wall = data['last'][0] - data['first'][0]
    sim = data['last'][1] - data['first'][1]
    print(f'시뮬레이션 속도 {sim / wall:.2f}배 (1.00 = 실시간), '
          f'/scan {data["scans"] / wall:.1f} Hz (실제 시간 기준)')
    return 0 if data['scans'] else 1


if __name__ == '__main__':
    sys.exit(main())
