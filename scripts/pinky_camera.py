#!/usr/bin/env python3
"""
로봇에서 실행: 카메라 영상을 웹 주소로 내보낸다 (MJPEG). 관제센터 화면이 이 주소를 그대로 보여 준다.

공식 실물 bringup 에는 카메라 노드가 없고, 영상을 ROS 토픽으로 와이파이에 실으면 무겁다.
그래서 로봇이 직접 작은 웹 서버로 내보내고 브라우저가 받아 본다. ROS 와 무관하다.

  python3 pinky_camera.py                       # http://<로봇 IP>:8090/stream
  python3 pinky_camera.py --port 8090 --width 640 --height 480 --fps 10 --flip

  /stream          이어지는 영상 (관제 화면의 <img> 가 받는다). 찾은 마커에 테두리와 번호를 그린다
  /snapshot.jpg    사진 한 장
  /markers         지금 보이는 ArUco 마커 (JSON): 번호와 네 꼭짓점의 화면 좌표. 위치 보정에 쓴다

관제 서버에는 주소를 알려 준다:
  ros2 launch pinky_parking_control control.launch.py cameras:=pinky1=http://<로봇 IP>:8090/stream
카메라는 한 프로그램만 쓸 수 있다 (Pinky Studio 의 카메라, 주피터 예제와 동시에 못 쓴다).
"""
import argparse
import json
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
import threading
import time

import cv2
from picamera2 import Picamera2

STRETCH = 2      # 납작한 마커를 찾을 때 영상을 세로로 늘리는 배수


def make_detector(dictionary):
    """
    OpenCV 버전에 맞는 ArUco 검출 함수 (회색 영상 → 꼭짓점들, 번호들). 없으면 None.

    카메라가 낮아 바닥의 마커가 아주 납작하게 보인다. 보통 설정으로 못 찾으면 영상을 세로로
    2배 늘려(마커를 펴서) 한 번 더 찾고, 찾은 꼭짓점은 원래 좌표로 되돌린다.
    """
    if not hasattr(cv2, 'aruco'):
        return None
    table = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    new_api = hasattr(cv2.aruco, 'ArucoDetector')                 # OpenCV 4.7 이상

    def parameters(flat):
        p = cv2.aruco.DetectorParameters() if new_api else cv2.aruco.DetectorParameters_create()
        if flat:
            p.perspectiveRemoveIgnoredMarginPerCell = 0.3    # 칸 가장자리가 번져도 가운데만 본다
            p.adaptiveThreshConstant = 12
        return p

    def finder(flat):
        if new_api:
            detector = cv2.aruco.ArucoDetector(table, parameters(flat))
            return lambda gray: detector.detectMarkers(gray)[:2]
        params = parameters(flat)
        return lambda gray: cv2.aruco.detectMarkers(gray, table, parameters=params)[:2]

    normal, flat = finder(False), finder(True)

    def detect(gray):
        corners, ids = normal(gray)
        if ids is not None:
            return corners, ids
        tall = cv2.resize(gray, None, fx=1, fy=STRETCH, interpolation=cv2.INTER_LINEAR)
        corners, ids = flat(tall)
        if ids is None:
            return corners, None
        fixed = []
        for c in corners:
            c = c.copy()
            c[..., 1] /= STRETCH
            fixed.append(c)
        return fixed, ids
    return detect


class Frames:
    """카메라에서 계속 찍어 가장 최근 JPEG 한 장과 그 안의 마커를 들고 있는다."""

    def __init__(self, width, height, fps, flip, quality, dictionary='DICT_4X4_50'):
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_video_configuration(
            main={'size': (width, height), 'format': 'RGB888'}))
        self.camera.start()
        self.period, self.flip, self.quality = 1.0 / fps, flip, quality
        self.jpeg, self.changed = b'', threading.Condition()
        self.size = (width, height)
        self.detect = make_detector(dictionary)
        self.markers, self.stamp = [], 0.0
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            start = time.monotonic()
            frame = self.camera.capture_array()          # RGB888 설정은 OpenCV 의 BGR 순서로 나온다
            if self.flip:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            markers = []
            if self.detect is not None:
                corners, ids = self.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                if ids is not None:
                    frame = frame.copy()
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    markers = [{'id': int(i), 'corners': c.reshape(4, 2).round(2).tolist()}
                               for c, i in zip(corners, ids.flatten())]
            ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                with self.changed:
                    self.jpeg = data.tobytes()
                    self.markers, self.stamp = markers, time.time()
                    self.changed.notify_all()
            time.sleep(max(0.0, self.period - (time.monotonic() - start)))

    def wait(self):
        with self.changed:
            self.changed.wait(2.0)
            return self.jpeg


def handler(frames):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):  # noqa: N802  http.server 가 정한 이름
            if self.path.startswith('/markers'):
                body = json.dumps({'stamp': frames.stamp, 'width': frames.size[0],
                                   'height': frames.size[1], 'markers': frames.markers}).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path.startswith('/snapshot'):
                data = frames.wait()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)
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
    parser = argparse.ArgumentParser(description='Pinky 카메라 영상 내보내기 (MJPEG)')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=10.0)
    parser.add_argument('--quality', type=int, default=70, help='JPEG 품질 1~100')
    parser.add_argument('--flip', action='store_true', help='영상이 거꾸로면 180° 돌린다')
    parser.add_argument('--dictionary', default='DICT_4X4_50', help='ArUco 사전')
    args = parser.parse_args()
    frames = Frames(args.width, args.height, args.fps, args.flip, args.quality, args.dictionary)
    print(f'카메라 영상: http://0.0.0.0:{args.port}/stream', flush=True)
    ThreadingHTTPServer(('0.0.0.0', args.port), handler(frames)).serve_forever()


if __name__ == '__main__':
    main()
