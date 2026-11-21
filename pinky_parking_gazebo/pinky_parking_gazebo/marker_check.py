"""
시뮬레이션 카메라 영상에서 ArUco 마커가 보이는지 확인한다 (그 로봇의 도메인에서).

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_gazebo marker_check
  ROS_DOMAIN_ID=11 ros2 run pinky_parking_gazebo marker_check --seconds 5 --save /tmp/view.png

/camera/image_raw 를 몇 초 받아, 영상마다 찾은 마커 번호와 화면 위치를 모아 출력한다.
바닥 마커는 낮은 카메라에서 납작하게 보이므로, 영상을 세로로 늘려 한 번 더 찾는다
(로봇의 scripts/pinky_camera.py 와 같은 방법). 마커를 하나도 못 찾으면 종료 코드 1.
"""
import argparse
import sys
import time

STRETCH = 2      # 납작한 마커를 찾을 때 영상을 세로로 늘리는 배수 (pinky_camera.py 와 같음)


def make_detector(dictionary='DICT_4X4_50'):
    """회색 영상 → [(번호, (가운데 u, v))]. OpenCV 4.6(옛 API)과 4.7 이상 모두 된다."""
    import cv2
    table = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(table, cv2.aruco.DetectorParameters())

        def find(gray):
            return detector.detectMarkers(gray)[:2]
    else:
        params = cv2.aruco.DetectorParameters_create()

        def find(gray):
            return cv2.aruco.detectMarkers(gray, table, parameters=params)[:2]

    def detect(gray):
        found = {}
        for stretch in (1, STRETCH):
            image = gray if stretch == 1 else cv2.resize(
                gray, None, fx=1, fy=stretch, interpolation=cv2.INTER_LINEAR)
            corners, ids = find(image)
            if ids is None:
                continue
            for c, marker_id in zip(corners, ids.flatten()):
                points = c.reshape(4, 2)
                center = (float(points[:, 0].mean()), float(points[:, 1].mean()) / stretch)
                found.setdefault(int(marker_id), center)
        return sorted(found.items())
    return detect


def image_to_gray(msg):
    """sensor_msgs/Image (rgb8, bgr8, mono8) → 회색 numpy 영상."""
    import cv2
    import numpy as np
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f'모르는 영상 형식입니다: {msg.encoding}')
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    image = data[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if channels == 1:
        return image[:, :, 0]
    code = cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(image, code)


def main():
    parser = argparse.ArgumentParser(description='카메라 영상의 ArUco 마커 확인')
    parser.add_argument('--seconds', type=float, default=3.0, help='받는 시간 [s, 실제 시간]')
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--dictionary', default='DICT_4X4_50')
    parser.add_argument('--save', default='', help='마지막 영상을 이 파일(PNG)로 저장')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    detect = make_detector(args.dictionary)
    seen, frames, last = {}, 0, None

    def on_image(msg):
        nonlocal frames, last
        gray = image_to_gray(msg)
        frames += 1
        last = gray
        for marker_id, center in detect(gray):
            seen.setdefault(marker_id, []).append(center)

    rclpy.init()
    node = rclpy.create_node('marker_check')
    node.create_subscription(Image, args.topic, on_image, qos_profile_sensor_data)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if args.save and last is not None:
        import cv2
        cv2.imwrite(args.save, last)
    if frames == 0:
        print(f'영상이 오지 않았습니다: {args.topic} (카메라를 켰는지, 도메인이 맞는지)')
        sys.exit(1)
    if not seen:
        print(f'영상 {frames}장에서 마커를 찾지 못했습니다')
        sys.exit(1)
    print(f'영상 {frames}장')
    for marker_id, centers in sorted(seen.items()):
        u = sum(c[0] for c in centers) / len(centers)
        v = sum(c[1] for c in centers) / len(centers)
        print(f'  마커 {marker_id}: {len(centers)}장에서 보임, 화면 가운데 ({u:.0f}, {v:.0f}) px')


if __name__ == '__main__':
    main()
