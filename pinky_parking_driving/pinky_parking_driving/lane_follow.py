"""
카메라로 차선을 보고 따라 달린다.

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_follow
  ROS_DOMAIN_ID=11 ros2 run pinky_parking_driving lane_follow --speed 0.08 --timeout 120

대상 로봇은 ROS_DOMAIN_ID 로 고른다. /camera/image_raw 를 받아 /cmd_vel 을 낸다.
차선을 계속 못 찾으면 멈추고 종료 코드 2, --timeout 이 지나면 멈추고 0.
Ctrl-C(SIGINT), kill(SIGTERM), 터미널 닫힘(SIGHUP)도 멈춤 명령을 보낸 뒤 0 으로 끝난다.
바퀴 제어는 마지막 속도 명령을 유지하므로, 멈춤 명령 없이 끝나면 로봇이 계속 달린다.
"""
import argparse
import math
from pathlib import Path
import signal
import sys
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from pinky_parking_driving.lane import Camera
from pinky_parking_driving.lane import find_lane
from pinky_parking_driving.lane import GroundProjector
from pinky_parking_driving.lane import pursuit_curvature
from pinky_parking_driving.lane import tape_mask
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
import yaml


def load_config(path=None):
    path = path or Path(get_package_share_directory('pinky_parking_driving')) / 'config/lane.yaml'
    return yaml.safe_load(Path(path).read_text())


class LaneFollower(Node):

    def __init__(self, config):
        super().__init__('lane_follow')
        self.config = config
        cam = config['camera']
        self.camera = Camera(cam['height'], cam['x'], math.radians(cam['tilt_deg']),
                             math.radians(cam['hfov_deg']))
        self.projector = None
        self.bridge = CvBridge()
        self.lost = 0
        self.failed = False
        self.frames = 0
        self.last_log = time.time()
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 1)

    def on_image(self, msg):
        detect, lane, control = self.config['detect'], self.config['lane'], self.config['control']
        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        gray = cv2.resize(gray, None, fx=detect['scale'], fy=detect['scale'],
                          interpolation=cv2.INTER_AREA)
        if self.projector is None or self.projector.x.shape != gray.shape:
            self.projector = GroundProjector(self.camera, gray.shape[1], gray.shape[0],
                                             detect['max_range'])
        xs, ys = self.projector.points(tape_mask(gray, detect['dark_threshold']))
        result = find_lane(xs, ys, lane['width'], lane['near'], lane['lookahead'], lane['step'])

        twist = Twist()
        if result.found:
            self.lost = 0
            curvature = pursuit_curvature(result.target)
            speed = control['speed'] / (1 + control['curve_slowdown'] * abs(curvature))
            limit = control['max_angular']
            twist.linear.x = speed
            twist.angular.z = max(-limit, min(limit, speed * curvature))
        else:
            self.lost += 1
            if self.lost >= control['lost_frames']:
                self.get_logger().error('차선을 찾지 못해 멈춥니다')
                self.failed = True
        self.cmd.publish(twist)

        self.frames += 1
        if time.time() - self.last_log > 2.0:
            self.get_logger().info(
                f'{self.frames / (time.time() - self.last_log):.0f} fps, '
                f'중심 {result.near_offset * 1000:+.0f} mm, 띠 {result.slices}개')
            self.frames, self.last_log = 0, time.time()

    def stop(self):
        for _ in range(3):
            self.cmd.publish(Twist())
            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description='카메라 차선 추종')
    parser.add_argument('--config', help='설정 파일 (기본: config/lane.yaml)')
    parser.add_argument('--speed', type=float, help='직선 속도 [m/s] (설정 파일 값을 덮어씀)')
    parser.add_argument('--timeout', type=float, default=0.0, help='이 시간 뒤 정지 [s], 0 = 계속')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    config = load_config(args.config)
    if args.speed is not None:
        config['control']['speed'] = args.speed
    # rclpy 기본 신호 처리는 신호를 받자마자 통신을 닫아 멈춤 명령을 보낼 수 없다.
    # 신호는 직접 받아 표시만 하고, 루프를 빠져나온 뒤 멈춤 명령을 보낸다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop_signals = []
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda signum, frame: stop_signals.append(signum))
    node = LaneFollower(config)
    start = time.time()
    while not stop_signals and not node.failed:
        rclpy.spin_once(node, timeout_sec=0.1)
        if args.timeout and time.time() - start > args.timeout:
            break
    node.stop()
    node.destroy_node()
    rclpy.try_shutdown()
    return 2 if node.failed else 0


if __name__ == '__main__':
    sys.exit(main())
