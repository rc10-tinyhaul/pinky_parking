"""
경로를 따라 주행한다 (뼈대: config/route.yaml 의 경유점을 Nav2 로 차례로 지난다).

  ROS_DOMAIN_ID=12 ros2 run pinky_parking_driving lap
  ROS_DOMAIN_ID=12 ros2 run pinky_parking_driving lap --route lap

대상 로봇은 ROS_DOMAIN_ID 로 고른다 (pinky1 = 11, pinky2 = 12). 모든 경유점을 지나면 종료 코드 0.
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


def load_routes(path=None):
    path = path or Path(get_package_share_directory('pinky_parking_driving')) / 'config/route.yaml'
    return yaml.safe_load(Path(path).read_text())['routes']


def pose_in_map(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'   # stamp 0 = 가장 최근 TF 를 쓴다 (시뮬레이션 시간과 어긋나지 않게)
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return pose


def main():
    parser = argparse.ArgumentParser(description='경로를 따라 주행')
    parser.add_argument('--route', default='lap', help='경로 이름 (기본: lap)')
    parser.add_argument('--routes', help='경로 설정 파일 (기본: config/route.yaml)')
    parser.add_argument('--timeout', type=float, default=300.0, help='제한 시간 [s, 실제 시간]')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    routes = load_routes(args.routes)
    if args.route not in routes:
        print(f'없는 경로입니다: {args.route} (있는 경로: {", ".join(routes)})')
        return 1
    waypoints = [pose_in_map(p['x'], p['y'], p['yaw']) for p in routes[args.route]]

    rclpy.init()
    nav = BasicNavigator('lap')
    # localizer 를 amcl 로 두면 BasicNavigator 가 초기 위치 (0, 0) 을 보내 AMCL 위치를 덮어쓸 수 있다.
    # AMCL 초기 위치는 Nav2 를 띄울 때 이미 정해지므로, Nav2 가 켜질 때까지만 기다린다.
    nav.waitUntilNav2Active(localizer='robot_localization')

    start = time.time()
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        if time.time() - start > args.timeout:
            nav.cancelTask()
            print(f'{args.route}: 시간 초과 ({args.timeout:.0f}s)')
            return 2
        time.sleep(0.2)
    result = nav.getResult()
    print(f'{args.route}: {result.name}, 경유점 {len(waypoints)}개, {time.time() - start:.1f}s')
    nav.destroyNode()
    rclpy.shutdown()
    return 0 if result == TaskResult.SUCCEEDED else 3


if __name__ == '__main__':
    sys.exit(main())
