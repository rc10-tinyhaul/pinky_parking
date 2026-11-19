"""
속도 명령 감시: 0 이 아닌 속도 명령이 끊기면 로봇을 세운다.

공식 Pinky 구동 노드(pinky_bringup)는 cmd_vel 을 받으면 다음 명령이 올 때까지 그 속도를
계속 유지한다. 그래서 명령을 보내던 PC 프로그램이 죽거나 와이파이가 끊기면 로봇이 마지막
속도로 계속 달린다. 이 노드를 로봇에서 함께 띄우면, 0 이 아닌 명령 뒤에 timeout 초 동안
새 명령이 없을 때 정지 명령(0)을 같은 cmd_vel 로 낸다. 로봇 안에서 돌므로 통신이 끊겨도
동작한다. 공식 노드는 고치지 않는다.

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup cmd_vel_watchdog
  ROS_DOMAIN_ID=11 python3 cmd_vel_watchdog.py --timeout 0.5      (로봇에 이 파일만 복사해서)

rclpy 와 geometry_msgs 만 쓴다(이 저장소를 로봇에 설치하지 않아도 된다).
명령을 계속 보내는 프로그램(Nav2, lane_follow)과 함께 쓴다. 키보드로 몰 때는 키를 누른 동안
명령이 반복되게 한다: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0
"""
import argparse
import sys
import time


class Watchdog:
    """정지 판단 (ROS 와 무관). 시각은 초 단위 실수."""

    def __init__(self, timeout):
        self.timeout = timeout
        self.moving = False
        self.last = 0.0

    def on_command(self, linear, angular, now):
        """cmd_vel 을 받을 때마다 부른다. 정지 명령(0)이면 감시를 멈춘다(자기가 낸 0 포함)."""
        self.moving = abs(linear) > 1e-6 or abs(angular) > 1e-6
        self.last = now

    def should_stop(self, now):
        """주기적으로 부른다. True 이면 정지 명령을 내야 한다(한 번 True 뒤에는 새 명령 전까지 False)."""
        if self.moving and now - self.last > self.timeout:
            self.moving = False
            return True
        return False


def main():
    parser = argparse.ArgumentParser(description='속도 명령이 끊기면 로봇을 세움')
    parser.add_argument('--timeout', type=float, default=0.5,
                        help='이 시간 [s] 동안 새 명령이 없으면 정지')
    parser.add_argument('--topic', default='cmd_vel', help='속도 명령 토픽')
    import rclpy
    from geometry_msgs.msg import Twist
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node('cmd_vel_watchdog')
    watchdog = Watchdog(args.timeout)
    publisher = node.create_publisher(Twist, args.topic, 10)

    def on_twist(msg):
        watchdog.on_command(msg.linear.x, msg.angular.z, time.monotonic())

    def check():
        if watchdog.should_stop(time.monotonic()):
            node.get_logger().warn(f'{args.timeout:.1f}s 동안 속도 명령이 없어 로봇을 세웁니다')
            for _ in range(3):
                publisher.publish(Twist())

    node.create_subscription(Twist, args.topic, on_twist, 10)
    node.create_timer(0.05, check)
    node.get_logger().info(f'{args.topic} 감시 시작 (끊김 {args.timeout:.1f}s 이면 정지)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
