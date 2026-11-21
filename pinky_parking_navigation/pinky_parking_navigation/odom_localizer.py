"""
AMCL 대신 바퀴 기록(오도메트리)만으로 지도 위치를 정한다.

경기장(바닥 테이프, 마커)이 아직 없어 센서로 위치를 잡을 수 없을 때 쓰는 임시 방법이다.
로봇을 알려진 자리(예: 주차 칸)에 놓고 켜면, 그 자리를 지도 좌표 (x, y, yaw)로 보고
map -> odom 변환을 고정해 낸다. 오래 달리면 바퀴 미끄러짐만큼 오차가 쌓인다.

- TF: map -> odom (고정, /tf_static)
- amcl_pose: map 좌표의 로봇 위치 (AMCL 과 같은 형식이라 관제 서버가 그대로 읽는다)
- initialpose 를 받으면(RViz 의 2D Pose Estimate) 지금 위치를 그 값으로 다시 맞춘다
- SLAM 과 함께(localization:=slam): odom_frame:=slam, publish_pose:=false 로 띄우면 map -> slam 만 낸다.
  slam_toolbox 의 지도 프레임은 켤 때의 odom 프레임과 같으므로 같은 계산이 맞다 (odom 이 0 이 아니어도).

  ros2 run pinky_parking_navigation odom_localizer --ros-args -p x:=0.0 -p y:=0.15 -p yaw:=1.5708
"""
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros import TransformListener

POSE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """평면 변환 a ∘ b. 각각 (x, y, yaw)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by,
            math.atan2(math.sin(at + bt), math.cos(at + bt)))


def inverse(a):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, -t)


def map_to_odom(map_pose, odom_pose):
    """로봇의 map 위치와 odom 위치가 주어졌을 때 map -> odom 변환."""
    return compose(map_pose, inverse(odom_pose))


class OdomLocalizer(Node):
    """시작 위치 + 오도메트리 = 지도 위치."""

    def __init__(self):
        super().__init__('odom_localizer')
        start = tuple(float(self.declare_parameter(k, 0.0).value) for k in ('x', 'y', 'yaw'))
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        # False: 고정 변환만 낸다 (SLAM 과 함께 쓸 때. 위치는 tf_pose 가 낸다)
        self.publish_pose_on = bool(self.declare_parameter('publish_pose', True).value)
        self.start = start
        self.offset = None          # map -> odom. 첫 odom 을 받으면 정한다
        self.odom = None            # 마지막 odom 위치 (x, y, yaw)
        self.tf = StaticTransformBroadcaster(self)
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', POSE_QOS)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initial, 1)
        self.create_timer(0.1, self.publish_pose)
        self.get_logger().info(
            f'오도메트리 위치: 시작 = ({start[0]:.3f}, {start[1]:.3f}, {start[2]:.3f}). '
            'odom 을 기다립니다')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.offset is None:   # 켤 때 odom 이 0 이 아니어도(재시작 등) 지금 자리를 시작 위치로
            self.set_offset(map_to_odom(self.start, self.odom))

    def on_initial(self, msg):
        """'지금 여기 있다'(initialpose)를 받으면 그 자리가 되도록 고정 변환을 다시 잡는다."""
        if self.odom is None:
            return
        p = msg.pose.pose
        target = (p.position.x, p.position.y, yaw_of(p.orientation))
        current = self.odom
        if self.odom_frame != 'odom':
            # SLAM 과 함께 쓸 때: 우리 프레임(slam) 안의 로봇 위치는 SLAM 이 고친 값이라 TF 에서 읽는다
            try:
                t = self.buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
            except TransformException as error:
                self.get_logger().warn(f'위치를 다시 맞추지 못했습니다 (TF 없음): {error}')
                return
            current = (t.transform.translation.x, t.transform.translation.y,
                       yaw_of(t.transform.rotation))
        self.set_offset(map_to_odom(target, current))
        self.get_logger().info(f'위치를 다시 맞췄습니다: ({target[0]:.3f}, {target[1]:.3f})')

    def set_offset(self, offset):
        self.offset = offset
        x, y, yaw = offset
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = self.map_frame, self.odom_frame
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(t)

    def publish_pose(self):
        if self.offset is None or self.odom is None or not self.publish_pose_on:
            return
        x, y, yaw = compose(self.offset, self.odom)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = OdomLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
