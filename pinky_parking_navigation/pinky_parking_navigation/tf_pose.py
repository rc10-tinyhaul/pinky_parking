"""
TF 의 map -> base_footprint 를 amcl_pose 토픽으로 낸다.

SLAM(slam_toolbox)으로 위치를 잡을 때는 AMCL 이 없어 amcl_pose 가 나오지 않는다.
관제 서버와 다른 도구가 AMCL 때와 같은 방식으로 위치를 읽도록 TF 를 같은 형식으로 옮긴다.

  ros2 run pinky_parking_navigation tf_pose
"""
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

POSE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TfPose(Node):
    """map 좌표의 로봇 위치를 10 Hz 로 낸다."""

    def __init__(self):
        super().__init__('tf_pose')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', POSE_QOS)
        self.create_timer(0.1, self.publish)

    def publish(self):
        try:
            t = self.buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return                      # 아직 위치를 못 잡았다
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.orientation = t.transform.rotation
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TfPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
