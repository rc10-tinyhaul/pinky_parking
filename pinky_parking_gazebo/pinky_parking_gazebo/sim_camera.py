"""
시뮬레이션 카메라 서버: 로봇의 scripts/pinky_camera.py 와 같은 주소로 영상과 마커를 낸다.

실물에서는 로봇의 카메라 서버가 /markers(JSON)를 내고, PC 의 marker_localizer 가 그것을 읽어 위치를
바로잡는다. 시뮬레이션에도 같은 서버를 두면 marker_localizer 를 실물과 똑같이 쓸 수 있다.
그 로봇의 도메인에서 /camera/image_raw, /camera/camera_info 를 받는다.

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_gazebo sim_camera --ros-args -p port:=8011

  /markers         지금 보이는 ArUco 마커 (JSON): 번호와 네 꼭짓점의 화면 좌표. 실물 서버와 같은 모양에
                   camera(초점 거리, 화면 중심, 로봇 중심에서 카메라까지 앞쪽 거리)를 더 싣는다
  /snapshot.jpg    사진 한 장
  /stream          이어지는 영상 (MJPEG)
"""
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
import json
import threading
import time

from pinky_parking_gazebo.marker_check import image_to_gray
from pinky_parking_gazebo.marker_check import make_corner_detector

CAMERA_FORWARD = 0.035   # base_footprint → 카메라 렌즈 앞쪽 거리 [m] (공식 URDF: 0.020 + 0.015 cos 8°)


def markers_reply(frames):
    """/markers 응답 (dict). 실물 서버의 stamp, width, height, markers 에 camera 를 더한다."""
    return {'stamp': frames.stamp, 'width': frames.size[0], 'height': frames.size[1],
            'markers': frames.markers, 'camera': frames.camera}


class Frames:
    """가장 최근 영상(JPEG)과 그 안의 마커. ROS 콜백이 채우고 HTTP 가 읽는다."""

    def __init__(self, dictionary='DICT_4X4_50', forward=CAMERA_FORWARD, quality=70):
        self.detect = make_corner_detector(dictionary)
        self.forward, self.quality = forward, quality
        self.jpeg, self.changed = b'', threading.Condition()
        self.size, self.markers, self.stamp = (0, 0), [], 0.0
        self.camera = None

    def set_info(self, fx, cx, cy):
        self.camera = {'focal_px': round(fx, 2), 'cx': round(cx, 2), 'cy': round(cy, 2),
                       'forward': self.forward}

    def update(self, msg):
        import cv2
        gray = image_to_gray(msg)
        found = self.detect(gray)
        markers = [{'id': marker_id, 'corners': corners}
                   for marker_id, corners in sorted(found.items())]
        frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for m in markers:
            pts = [(int(u), int(v)) for u, v in m['corners']]
            for a, b in zip(pts, pts[1:] + pts[:1]):
                cv2.line(frame, a, b, (0, 200, 0), 2)
            cv2.putText(frame, str(m['id']), pts[0], cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        with self.changed:
            if ok:
                self.jpeg = data.tobytes()
            self.size, self.markers, self.stamp = (msg.width, msg.height), markers, time.time()
            self.changed.notify_all()

    def wait(self):
        with self.changed:
            self.changed.wait(2.0)
            return self.jpeg


def handler(frames):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send_body(self, content_type, body):
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802  http.server 가 정한 이름
            if self.path.startswith('/markers'):
                self.send_body('application/json', json.dumps(markers_reply(frames)).encode())
                return
            if self.path.startswith('/snapshot'):
                self.send_body('image/jpeg', frames.wait())
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                while True:
                    data = frames.wait()
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs.msg import Image

    class SimCamera(Node):
        def __init__(self):
            super().__init__('sim_camera')
            port = int(self.declare_parameter('port', 8011).value)
            rate = float(self.declare_parameter('rate_hz', 5.0).value)
            self.frames = Frames(self.declare_parameter('dictionary', 'DICT_4X4_50').value,
                                 float(self.declare_parameter('forward', CAMERA_FORWARD).value))
            self.period, self.last = 1.0 / rate, 0.0
            self.create_subscription(Image, 'camera/image_raw', self.on_image,
                                     qos_profile_sensor_data)
            self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info,
                                     qos_profile_sensor_data)
            server = ThreadingHTTPServer(('0.0.0.0', port), handler(self.frames))
            threading.Thread(target=server.serve_forever, daemon=True).start()
            self.get_logger().info(f'시뮬레이션 카메라 서버: http://localhost:{port}/markers')

        def on_info(self, msg):
            if self.frames.camera is None and msg.k[0] > 0:
                self.frames.set_info(msg.k[0], msg.k[2], msg.k[5])

        def on_image(self, msg):
            now = time.monotonic()
            if now - self.last < self.period:      # 마커 찾기는 무거워 초당 rate_hz 장만
                return
            self.last = now
            self.frames.update(msg)

    rclpy.init()
    node = SimCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
