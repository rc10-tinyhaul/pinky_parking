"""
로봇 한 대와의 연결: 그 로봇의 ROS_DOMAIN_ID 에서 도는 rclpy Context, 노드, 실행기.

Nav2 액션(navigate_to_pose, follow_waypoints)과 /tf 는 도메인을 건너지 못하므로,
관제 서버는 로봇마다 그 도메인의 Context 를 따로 만들어 직접 붙는다.
한 프로세스 안에서 도메인 11, 12 … 에 동시에 참여한다.
"""
from collections import deque
import json
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from pinky_parking_control import health
from pinky_parking_control.mapdata import encode_grid
from pinky_parking_control.mapdata import scan_points
from pinky_parking_control.model import CANCELED
from pinky_parking_control.model import FAILED
from pinky_parking_control.model import NotReady
from pinky_parking_control.model import quaternion_to_yaw
from pinky_parking_control.model import SUCCEEDED
from pinky_parking_control.model import TaskBusy
from pinky_parking_control.model import yaw_to_quaternion
from pinky_parking_control.usage import Usage
from rcl_interfaces.msg import Log
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# AMCL 은 amcl_pose 를 transient_local 로 낸다: 늦게 붙어도 마지막 위치를 받는다
AMCL_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
POSE_FRESH_S = 5.0   # 이보다 오래된 위치는 '위치 모름'으로 본다
LATCHED = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
SLAM_FRAME = 'slam'          # localization:=slam 일 때 SLAM 지도의 프레임 (map -> slam 은 고정 변환)


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
        self.teleop_time = 0.0   # 마지막 직접 운전 명령 시각 (0 = 직접 운전 아님)
        self.speed = {'linear': 0.0, 'angular': 0.0}
        self.battery_v = None
        self.battery_pct = None                   # 배터리 % (실물 battery/percent)
        self.usage = Usage()                      # 달린 거리, 바퀴 사용량, 배터리 사용 (서버 켠 뒤)
        self.camera_url = None                    # 실물 로봇의 카메라 영상 주소 (서버 설정)
        self.slam_map, self.slam_seq = None, 0    # SLAM 지도 (localization:=slam 일 때)
        self.slam_tf = None                       # map -> slam 고정 변환 {x, y, yaw}
        self.scan = None                          # 마지막 라이다 스캔
        self._stamps = {'odom': deque(maxlen=40), 'scan': deque(maxlen=20)}   # 수신 시각 (Hz 계산)

        self.context = Context()
        rclpy.init(context=self.context, domain_id=self.domain_id)
        self.node = Node(f'control_{name}', context=self.context)
        self.node.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, AMCL_QOS)
        self.node.create_subscription(Odometry, 'odom', self._on_odom, 10)
        # 실물 로봇의 배터리 전압 (공식 battery_publisher). 시뮬레이션에는 없어 None 으로 남는다
        self.node.create_subscription(Float32, 'battery/voltage', self._on_battery, 1)
        self.node.create_subscription(Float32, 'battery/percent', self._on_battery_pct, 1)
        self.node.create_subscription(OccupancyGrid, '/slam_map', self._on_slam_map, LATCHED)
        self.node.create_subscription(TFMessage, '/tf_static', self._on_tf_static, LATCHED)
        self.node.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.initialpose = self.node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        # 카메라 마커 위치 보정 (marker_localizer 가 떠 있을 때): 상태를 받고 등록을 시킨다
        self.marker = None
        self.node.create_subscription(String, 'marker_status', self._on_marker, LATCHED)
        self.marker_register = self.node.create_publisher(String, 'marker_register', 1)
        # Nav2 가 낸 경고·오류 (왜 안 움직이는지 알려 주는 데 쓴다)
        self.issue = None                         # {'text': 사유, 'raw': 원문, 'time': 받은 시각}
        self.node.create_subscription(Log, '/rosout', self._on_log, 50)
        self.param_clients = {}                   # (노드, set|get) → 서비스 클라이언트
        self.cmd_vel = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.clients = {
            'pose': ActionClient(self.node, NavigateToPose, 'navigate_to_pose'),
            'waypoints': ActionClient(self.node, FollowWaypoints, 'follow_waypoints'),
            'path': ActionClient(self.node, FollowPath, 'follow_path'),
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
            task = self.task
            if task is not None and not task.done:      # 궤적 기록 (보정·분석용)
                row = [self.pose_time - task.started, p.x, p.y, self.pose['yaw'],
                       self.speed['linear'], self.speed['angular']]
                task.trail.append([round(v, 4) for v in row])

    def _on_odom(self, msg):
        twist = msg.twist.twist
        with self._lock:
            self.speed = {'linear': twist.linear.x, 'angular': twist.angular.z}
            now = time.time()
            self._stamps['odom'].append(now)
            self.usage.on_odom(now, twist.linear.x, twist.angular.z)

    def _on_slam_map(self, msg):
        grid = encode_grid(msg)
        with self._lock:
            self.slam_map, self.slam_seq = grid, self.slam_seq + 1

    def _on_tf_static(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == SLAM_FRAME:
                q = t.transform.rotation
                with self._lock:
                    self.slam_tf = {'x': t.transform.translation.x, 'y': t.transform.translation.y,
                                    'yaw': quaternion_to_yaw(q.x, q.y, q.z, q.w)}

    def _on_scan(self, msg):
        with self._lock:
            self.scan = msg
            self._stamps['scan'].append(time.time())

    def _on_log(self, msg):
        if msg.level < Log.WARN:
            return
        reason = health.reason_from_log(msg.msg)
        if reason:
            with self._lock:
                self.issue = {'text': reason, 'raw': msg.msg[:200], 'time': time.time()}

    def _recent_reason(self, now):
        with self._lock:
            issue = self.issue
        return issue['text'] if issue and now - issue['time'] <= health.REASON_FRESH_S else ''

    def _on_marker(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self.marker = {**data, 'received': time.time()}

    def register_markers(self, pose):
        """지금 보이는 마커를 등록시킨다: pose = 로봇의 진짜 위치 (예: 주차 칸 가운데)."""
        self.marker_register.publish(String(data=json.dumps(pose)))

    def _on_battery(self, msg):
        value = float(msg.data)
        if not 3.0 < value < 12.0:       # 가끔 0.5 V 처럼 튀는 값이 온다 (읽기 오류): 버린다
            return
        with self._lock:
            self.battery_v = value
            self.usage.on_battery(value)

    def _on_battery_pct(self, msg):
        value = float(msg.data)
        if 0.0 <= value <= 100.0:
            with self._lock:
                self.battery_pct = value

    def _on_feedback(self, task, feedback_msg):
        feedback = feedback_msg.feedback
        with self._lock:
            if task is not self.task:
                return
            if hasattr(feedback, 'distance_remaining'):
                task.distance_remaining = feedback.distance_remaining
            if hasattr(feedback, 'distance_to_goal'):       # follow_path
                task.distance_remaining = feedback.distance_to_goal
            if hasattr(feedback, 'current_waypoint'):
                task.waypoint_index = int(feedback.current_waypoint)

    def _on_goal_response(self, task, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            reason = self._recent_reason(time.time())
            self._finish(task, FAILED, f'Nav2 가 목표를 거절했습니다: {reason}' if reason
                         else 'Nav2 가 목표를 거절했습니다')
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
            code = getattr(response.result, 'error_code', 0) if response else 0
            self._finish(task, FAILED,
                         health.failure_detail(code, self._recent_reason(time.time())))

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
        """
        작업을 Nav2 에 보낸다.

        task.mode 가 'path' 면 poses 를 이어진 경로로 보고 follow_path (멈추지 않고 따라간다).
        아니면 poses 가 하나일 때 navigate_to_pose, 여럿일 때 follow_waypoints (점마다 도착 판정).
        """
        kind = task.mode or ('pose' if len(task.poses) == 1 else 'waypoints')
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
        elif kind == 'path':
            goal = FollowPath.Goal()
            goal.path = Path()
            goal.path.header.frame_id = 'map'
            goal.path.poses = [pose_stamped(p) for p in task.poses]
            goal.controller_id, goal.goal_checker_id = 'FollowPath', 'general_goal_checker'
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
        with self._lock:
            self.teleop_time = 0.0
        self.cmd_vel.publish(Twist())
        return canceled

    def teleop(self, linear, angular):
        """직접 운전 속도를 낸다. Nav2 작업 중이면 거절한다 (둘이 cmd_vel 을 다투지 않게)."""
        with self._lock:
            if self.task is not None and not self.task.done:
                raise TaskBusy(f'{self.name} 은(는) {self.task.action} 작업 중입니다. 먼저 취소하세요')
            self.teleop_time = time.time()
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd_vel.publish(msg)
        return {'linear': msg.linear.x, 'angular': msg.angular.z}

    def teleop_watchdog(self, now, timeout_s):
        """직접 운전 명령이 timeout_s 동안 끊기면 한 번 멈춘다."""
        with self._lock:
            stale = self.teleop_time and now - self.teleop_time > timeout_s
            if stale:
                self.teleop_time = 0.0
        if stale:
            self.cmd_vel.publish(Twist())

    # ---- 진단·설정 (API 스레드) ----------------------------------------
    def _rate(self, key, now):
        """최근 2초 동안 받은 주기 [Hz]. 끊겼으면 0."""
        recent = [t for t in self._stamps[key] if now - t < 2.0]
        return round(len(recent) / 2.0, 1)

    def slam_view(self):
        """SLAM 지도와 map -> slam 변환. 없으면 None."""
        with self._lock:
            if self.slam_map is None:
                return None
            return {**self.slam_map, 'seq': self.slam_seq,
                    'transform': dict(self.slam_tf or {'x': 0.0, 'y': 0.0, 'yaw': 0.0})}

    def scan_view(self):
        """지금 라이다가 보는 점들 (지도 좌표)."""
        with self._lock:
            scan, pose = self.scan, dict(self.pose) if self.pose else None
        if scan is None or pose is None:
            return {'points': []}
        return {'points': scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                      min(scan.range_max, 8.0), pose)}

    def set_pose(self, pose):
        """로봇에게 '지금 여기 있다'고 알려 준다 (initialpose: AMCL, odom_localizer 가 받는다)."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose = pose_stamped(pose).pose
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        self.initialpose.publish(msg)

    def _param_client(self, node_name, kind):
        key = (node_name, kind)
        if key not in self.param_clients:
            srv = SetParameters if kind == 'set' else GetParameters
            self.param_clients[key] = self.node.create_client(
                srv, f'/{node_name}/{kind}_parameters')
        return self.param_clients[key]

    def set_nav2_parameters(self, by_node):
        """{노드: {파라미터: 값}} 을 Nav2 에 보낸다 (달리는 중에도 적용). 보낸 노드 이름을 돌려준다."""
        sent = []
        for node_name, params in by_node.items():
            client = self._param_client(node_name, 'set')
            if not client.service_is_ready():
                continue
            request = SetParameters.Request()
            for name, value in params.items():
                if isinstance(value, (list, tuple)):
                    pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                        double_array_value=[float(v) for v in value])
                else:
                    pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                        double_value=float(value))
                request.parameters.append(Parameter(name=name, value=pv))
            client.call_async(request)
            sent.append(node_name)
        return sent

    def get_nav2_parameters(self, sources, timeout_s=1.0):
        """{키: (노드, 파라미터)} 의 지금 값을 읽는다. 못 읽은 키는 빠진다."""
        by_node = {}
        for key, (node_name, param) in sources.items():
            by_node.setdefault(node_name, []).append((key, param))
        out, waiting = {}, []
        for node_name, items in by_node.items():
            client = self._param_client(node_name, 'get')
            if not client.service_is_ready():
                continue
            done = threading.Event()
            future = client.call_async(GetParameters.Request(names=[p for _, p in items]))
            future.add_done_callback(lambda _f, d=done: d.set())
            waiting.append((items, future, done))
        deadline = time.time() + timeout_s
        for items, future, done in waiting:
            if not done.wait(max(0.0, deadline - time.time())) or future.result() is None:
                continue
            for (key, _), value in zip(items, future.result().values):
                if value.type == ParameterType.PARAMETER_DOUBLE:
                    out[key] = value.double_value
        return out

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
                'teleop': bool(self.teleop_time),
                'battery_v': round(self.battery_v, 2) if self.battery_v is not None else None,
                'battery_pct': round(self.battery_pct) if self.battery_pct is not None else None,
                'usage': self.usage.to_dict(self.battery_v),
                'odom_hz': self._rate('odom', now), 'scan_hz': self._rate('scan', now),
                'slam_seq': self.slam_seq, 'camera_url': self.camera_url,
                'issue': ({'text': self.issue['text'], 'raw': self.issue['raw'],
                           'age_s': round(now - self.issue['time'], 1)} if self.issue else None),
                'marker': (dict(self.marker) if self.marker
                           and now - self.marker['received'] < 3.0 else None),
                'task': self.task.to_dict(now) if self.task else None,
            }

    def close(self):
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if self.context.ok():
            rclpy.shutdown(context=self.context)
        self._thread.join(timeout=1.0)
