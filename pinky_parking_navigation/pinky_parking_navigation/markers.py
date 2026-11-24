"""
바닥(또는 벽)의 ArUco 마커로 로봇 위치를 계산한다. ROS 없이 시험할 수 있다.

카메라가 마커의 네 꼭짓점을 보면(화면 좌표) 마커가 로봇 기준 어디에 있는지 알 수 있다.
마커의 지도 좌표를 알면 거꾸로 로봇의 지도 위치가 나온다:

    로봇 위치 = 마커의 지도 위치 ∘ (로봇 기준 마커 위치)⁻¹

마커의 지도 위치는 자로 재지 않고 '등록'한다: 로봇을 아는 자리(주차 칸)에 놓고 마커를 보게 한 뒤
    마커의 지도 위치 = 로봇 위치 ∘ 로봇 기준 마커 위치
로 정한다. 카메라 값(초점 거리, 앞쪽 거리)이 조금 틀려도 등록할 때와 볼 때 같은 값을 쓰므로,
등록한 자리 근처에서는 오차가 서로 지워진다.

위치는 모두 (x, y, yaw) 이고 평면 위다. 마커의 yaw 는 마커의 오른쪽(+x) 방향이다.
"""
import math


def compose(a, b):
    """평면 변환 a ∘ b."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by,
            math.atan2(math.sin(at + bt), math.cos(at + bt)))


def inverse(a):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, -t)


def marker_in_robot(corners, size, focal, center, camera_forward=0.0):
    """
    화면의 마커 꼭짓점 네 개 → 로봇 기준 마커 위치.

    corners: [[u, v] x 4] 왼쪽 위부터 시계 방향 (ArUco 순서). size: 마커 한 변 [m].
    focal: 초점 거리 [px]. center: 화면 중심 (cx, cy). camera_forward: 로봇 중심에서 카메라까지 앞쪽 거리 [m].
    돌려준다: {'pose': (앞, 왼쪽, yaw), 'height': 카메라가 마커 면에서 떨어진 높이, 'distance': 앞쪽 거리}.
    마커가 놓인 면(바닥)을 기준으로 앞·왼쪽을 정하므로 카메라의 높이와 기울기는 몰라도 된다.
    """
    import cv2
    import numpy as np
    half = size / 2.0
    points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
                      dtype=np.float64)
    camera = np.array([[focal, 0, center[0]], [0, focal, center[1]], [0, 0, 1]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(points, np.array(corners, dtype=np.float64), camera, None,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)
    normal = rotation[:, 2]                       # 마커 면에서 카메라 쪽으로 나오는 방향
    if normal.dot(t) > 0:
        normal = -normal
    optical = np.array([0.0, 0.0, 1.0])
    forward = optical - optical.dot(normal) * normal
    if np.linalg.norm(forward) < 1e-6:            # 마커를 정면으로 내려다봄: 앞 방향을 정할 수 없다
        return None
    forward /= np.linalg.norm(forward)
    left = np.cross(normal, forward)
    axis = rotation[:, 0]                         # 마커의 오른쪽(+x)
    ahead = float(t.dot(forward))
    return {'pose': (camera_forward + ahead, float(t.dot(left)),
                     math.atan2(float(axis.dot(left)), float(axis.dot(forward)))),
            'height': float(-t.dot(normal)), 'distance': ahead}


def robot_from_marker(marker_map, marker_robot):
    """마커의 지도 위치와 로봇 기준 마커 위치 → 로봇의 지도 위치."""
    return compose(marker_map, inverse(marker_robot))


def marker_from_robot(robot_map, marker_robot):
    """등록: 로봇의 지도 위치와 로봇 기준 마커 위치 → 마커의 지도 위치."""
    return compose(robot_map, marker_robot)


def median_pose(poses):
    """여러 번 본 값의 가운데 값 (x, y 는 중앙값, yaw 는 방향 평균)."""
    xs = sorted(p[0] for p in poses)
    ys = sorted(p[1] for p in poses)
    yaw = math.atan2(sum(math.sin(p[2]) for p in poses), sum(math.cos(p[2]) for p in poses))
    return (xs[len(xs) // 2], ys[len(ys) // 2], yaw)


def difference(a, b):
    """두 위치의 거리 [m] 와 방향 차이 [rad]."""
    return (math.hypot(a[0] - b[0], a[1] - b[1]),
            abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))))


def detection(marker_id, in_robot, registered=False):
    """
    로봇 기준 마커 하나 → 토픽에 싣는 값 (marker_detections).

    x: 앞 [m], y: 왼쪽 [m], yaw: 마커 오른쪽 변의 방향 [rad], distance: 로봇 중심에서 마커 가운데까지 [m],
    bearing: 마커가 보이는 방향 [rad] (0 = 정면, + = 왼쪽).
    """
    x, y, yaw = in_robot
    return {'id': int(marker_id), 'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4),
            'distance': round(math.hypot(x, y), 4), 'bearing': round(math.atan2(y, x), 4),
            'registered': bool(registered)}


def camera_from_reply(reply, focal, center, forward):
    """
    카메라 서버 응답에 camera(초점 거리, 화면 중심, 앞쪽 거리)가 있으면 그 값을, 없으면 설정 값을 쓴다.

    시뮬레이션 카메라 서버(pinky_parking_gazebo/sim_camera.py)는 Gazebo 카메라 값을 싣고, 실물 서버는
    싣지 않으므로 markers.yaml 의 camera 값을 쓴다.
    """
    camera = reply.get('camera') or {}
    if 'cx' in camera and 'cy' in camera:
        center = (float(camera['cx']), float(camera['cy']))
    return (float(camera.get('focal_px', focal)), center, float(camera.get('forward', forward)))


EDGE_MARGIN_PX = 4      # 꼭짓점이 화면 가장자리에서 이만큼 안이면 마커가 잘렸을 수 있다


def fully_visible(corners, width, height, margin=EDGE_MARGIN_PX):
    """
    마커 네 꼭짓점이 모두 화면 안쪽에 있는지.

    마커가 가까워져 화면 가장자리(바닥 마커는 주로 아래)에 걸리면 일부만 찍히는데, 검출기는 잘린 자리를
    꼭짓점으로 잡아 위치가 크게 틀린다(시뮬레이션에서 방향 28° 까지). 그런 마커는 쓰지 않는다.
    width, height 를 모르면(0) 거르지 않는다.
    """
    if not width or not height:
        return True
    return all(margin <= u <= width - 1 - margin and margin <= v <= height - 1 - margin
               for u, v in corners)
