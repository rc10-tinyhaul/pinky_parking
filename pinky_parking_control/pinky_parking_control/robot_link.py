"""
로봇 한 대와의 연결: 그 로봇의 ROS_DOMAIN_ID 에서 도는 rclpy Context, 노드, 실행기.

Nav2 액션(navigate_to_pose, follow_waypoints)과 /tf 는 도메인을 건너지 못하므로,
관제 서버는 로봇마다 그 도메인의 Context 를 따로 만들어 직접 붙는다.
한 프로세스 안에서 도메인 11, 12 … 에 동시에 참여한다.
"""
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from pinky_parking_control.model import CANCELED
from pinky_parking_control.model import FAILED
from pinky_parking_control.model import NotReady
from pinky_parking_control.model import quaternion_to_yaw
from pinky_parking_control.model import SUCCEEDED
from pinky_parking_control.model import TaskBusy
from pinky_parking_control.model import yaw_to_quaternion
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

# AMCL 은 amcl_pose 를 transient_local 로 낸다: 늦게 붙어도 마지막 위치를 받는다
AMCL_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
POSE_FRESH_S = 5.0   # 이보다 오래된 위치는 '위치 모름'으로 본다


def pose_stamped(pose):
    msg = PoseStamped()
    msg.header.frame_id = 'map'   # stamp 0 = 가장 최근 TF (시뮬레이션 시간과 어긋나지 않게)
    msg.pose.position.x, msg.pose.position.y = pose['x'], pose['y']
    q = yaw_to_quaternion(pose['yaw'])
    msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
    msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
    return msg


class RobotLink:
    """로봇 한 대. 작업은 한 번에 하나만 받는다."""

    def __init__(self, name, domain_id, on_finish):
        self.name, self.domain_id = name, int(domain_id)
        self._on_finish = on_finish
        self._lock = threading.Lock()
        self.task = None
        self._handle = None
        self._cancel_wanted = False
        self.pose, self.pose_time = None, 0.0
        self.speed = {'linear': 0.0, 'angular': 0.0}

        self.context = Context()
        rclpy.init(context=self.context, domain_id=self.domain_id)
        self.node = Node(f'control_{name}', context=self.context)
        self.node.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, AMCL_QOS)
        self.node.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.cmd_vel = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.clients = {
            'pose': ActionClient(self.node, NavigateToPose, 'navigate_to_pose'),
            'waypoints': ActionClient(self.node, FollowWaypoints, 'follow_waypoints'),
        }
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, name=f'ros-{name}', daemon=True)
        self._thread.start()

    # ---- ROS 콜백 (실행기 스레드) --------------------------------------
    def _spin(self):
        while self.context.ok():
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception as error:  # noqa: B902  한 번의 실패로 연결 전체가 멈추지 않게
                if self.context.ok():
                    self.node.get_logger().error(f'spin: {error}')

    def _on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self._lock:
            self.pose = {'x': p.x, 'y': p.y, 'yaw': quaternion_to_yaw(q.x, q.y, q.z, q.w)}
            self.pose_time = time.time()

    def _on_odom(self, msg):
        twist = msg.twist.twist
        with self._lock:
            self.speed = {'linear': twist.linear.x, 'angular': twist.angular.z}

    def _on_feedback(self, task, feedback_msg):
        feedback = feedback_msg.feedback
        with self._lock:
            if task is not self.task:
                return
            if hasattr(feedback, 'distance_remaining'):
                task.distance_remaining = feedback.distance_remaining
            if hasattr(feedback, 'current_waypoint'):
                task.waypoint_index = int(feedback.current_waypoint)

    def _on_goal_response(self, task, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._finish(task, FAILED, 'Nav2 가 목표를 거절했습니다')
            return
        with self._lock:
            if task is not self.task:
                return
            self._handle = handle
            cancel_now = self._cancel_wanted
        if cancel_now:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(task, f))

    def _on_result(self, task, future):
        response = future.result()
        status = response.status if response else GoalStatus.STATUS_UNKNOWN
        if status == GoalStatus.STATUS_SUCCEEDED:
            missed = list(getattr(response.result, 'missed_waypoints', []) or [])
            if missed:
                self._finish(task, FAILED, f'경유점 {len(missed)}개를 지나지 못했습니다')
            else:
                self._finish(task, SUCCEEDED)
        elif status == GoalStatus.STATUS_CANCELED:
            self._finish(task, CANCELED, task.detail or '취소했습니다')
        else:
            self._finish(task, FAILED, 'Nav2 가 작업을 중단했습니다')

    def _finish(self, task, state, detail=''):
        with self._lock:
            if not task.finish(state, detail):
                return
            if task is self.task:
                self._handle, self._cancel_wanted = None, False
        self._on_finish(task)

    # ---- 명령 (API 스레드) ---------------------------------------------
    def nav2_ready(self):
        return self.clients['pose'].server_is_ready()

    def start(self, task):
        """task.poses 가 하나면 navigate_to_pose, 여럿이면 follow_waypoints."""
        kind = 'pose' if len(task.poses) == 1 else 'waypoints'
        client = self.clients[kind]
        with self._lock:
            if self.task is not None and not self.task.done:
                raise TaskBusy(f'{self.name} 은(는) {self.task.action} 작업 중입니다')
            if not client.server_is_ready():
                raise NotReady(f'{self.name} 의 Nav2 가 아직 켜지지 않았습니다')
            self.task, self._handle, self._cancel_wanted = task, None, False
        if kind == 'pose':
            goal = NavigateToPose.Goal()
            goal.pose = pose_stamped(task.poses[0])
        else:
            goal = FollowWaypoints.Goal()
            goal.poses = [pose_stamped(p) for p in task.poses]
        future = client.send_goal_async(
            goal, feedback_callback=lambda f: self._on_feedback(task, f))
        future.add_done_callback(lambda f: self._on_goal_response(task, f))
        return task

    def cancel(self, reason='취소했습니다'):
        """진행 중인 작업을 취소한다. 취소할 작업이 없으면 False."""
        with self._lock:
            task = self.task
            if task is None or task.done:
                return False
            task.detail = reason
            handle, self._cancel_wanted = self._handle, True
        if handle is not None:
            handle.cancel_goal_async()
        return True

    def stop(self):
        """작업을 취소하고 바로 멈춘다 (속도 0)."""
        canceled = self.cancel('정지 명령')
        self.cmd_vel.publish(Twist())
        return canceled

    def status(self):
        now = time.time()
        with self._lock:
            fresh = self.pose is not None and now - self.pose_time < POSE_FRESH_S
            return {
                'name': self.name, 'domain_id': self.domain_id,
                'nav2_ready': self.nav2_ready(),
                'localized': fresh,
                'pose': dict(self.pose) if self.pose else None,
                'pose_age_s': round(now - self.pose_time, 1) if self.pose else None,
                'speed': {k: round(v, 3) for k, v in self.speed.items()},
                'task': self.task.to_dict(now) if self.task else None,
            }

    def close(self):
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if self.context.ok():
            rclpy.shutdown(context=self.context)
        self._thread.join(timeout=1.0)
