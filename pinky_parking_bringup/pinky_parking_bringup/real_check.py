"""
실물 로봇 연결 점검: 움직이기 전에 PC 에서 한 번 돌려 본다 (아무것도 움직이지 않는다).

  ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup real_check
  ROS_DOMAIN_ID=11 ros2 run pinky_parking_bringup real_check --seconds 8
  ROS_DOMAIN_ID=111 ros2 run pinky_parking_bringup real_check --bridged   (clock_bridge 를 쓸 때)

몇 초 동안 토픽을 듣고 아래를 본다. 문제(✗)가 있으면 종료 코드 1.
- /odom, /scan 이 오는지, 주기
- 같은 도메인에 다른 로봇이 섞였는지 (/odom 발행자가 둘 이상: 같은 와이파이의 다른 팀 로봇)
- 로봇과 PC 의 시계 차이 (크면 Nav2 가 TF 오류로 멈춘다)
- 속도 명령 감시 노드(cmd_vel_watchdog)가 떠 있는지
- 배터리 전압
"""
import argparse
import sys
import time

OK, WARN, BAD = '✓', '!', '✗'
CLOCK_WARN_S, CLOCK_BAD_S = 0.1, 0.5     # Nav2 의 TF 허용 지연은 0.1 ~ 0.2 s
BATTERY_WARN_V = 7.0                     # 공식 안내: 7 V 아래면 충전이 안 될 수 있다


def judge(odom_count, scan_count, seconds, odom_publishers, scan_publishers, clock_offset,
          watchdog, cmd_vel_subscribers, battery_v, bridged=False):
    """측정값 → [(표시, 항목, 설명)]. ROS 없이 시험할 수 있다."""
    rows = []
    hz = odom_count / seconds
    if odom_count == 0:
        rows.append((BAD, '오도메트리 /odom', '안 옵니다. 로봇에서 bringup 이 떠 있는지, 도메인·와이파이가 같은지'))
    else:
        rows.append((OK if hz > 10 else WARN, '오도메트리 /odom', f'{hz:.0f} Hz'))
    if scan_count == 0:
        rows.append((BAD, '라이다 /scan', '안 옵니다'))
    else:
        rows.append((OK, '라이다 /scan', f'{scan_count / seconds:.0f} Hz'))
    if odom_publishers > 1 or scan_publishers > 1:
        rows.append((BAD, '도메인 겹침', f'/odom 발행자 {odom_publishers}개, /scan 발행자 {scan_publishers}개: '
                     '다른 로봇이 같은 ROS_DOMAIN_ID 를 씁니다. 도메인을 바꾸세요'))
    elif odom_publishers == 1:
        rows.append((OK, '도메인 겹침', '이 도메인에 로봇 한 대'))
    if clock_offset is not None and bridged:
        rows.append((OK, '시계 차이', f'{clock_offset:+.1f} s: clock_bridge 가 고쳐서 넘깁니다'))
    elif clock_offset is not None:
        mark = OK if abs(clock_offset) < CLOCK_WARN_S else WARN \
            if abs(clock_offset) < CLOCK_BAD_S else BAD
        hint = '' if mark == OK else ' (로봇 시계를 맞추거나, 로봇을 도메인 + 100 에서 켜고 --bridged)'
        rows.append((mark, '시계 차이', f'{clock_offset * 1000:+.0f} ms{hint}'))
    if watchdog:
        rows.append((OK, '정지 감시 노드', 'cmd_vel_watchdog 실행 중'))
    else:
        rows.append((BAD, '정지 감시 노드', '없습니다. 로봇에서 cmd_vel_watchdog.py 를 켜세요 '
                     '(없으면 통신이 끊겨도 로봇이 계속 달립니다)'))
    if cmd_vel_subscribers == 0:
        rows.append((BAD, '구동 /cmd_vel', '받는 노드가 없습니다'))
    if battery_v is None:
        rows.append((WARN, '배터리', '전압이 안 옵니다 (battery/voltage)'))
    else:
        rows.append((OK if battery_v >= BATTERY_WARN_V else BAD, '배터리',
                     f'{battery_v:.2f} V' + ('' if battery_v >= BATTERY_WARN_V else ' : 충전하세요')))
    return rows


def main():
    parser = argparse.ArgumentParser(description='실물 로봇 연결 점검 (움직이지 않음)')
    parser.add_argument('--seconds', type=float, default=5.0, help='듣는 시간 [s]')
    parser.add_argument('--bridged', action='store_true',
                        help='로봇 도메인(도메인 + 100)에서 점검: 시계 차이는 clock_bridge 가 고친다')
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node('real_check')
    seen = {'odom': 0, 'scan': 0, 'offsets': [], 'battery': None}

    def on_odom(msg):
        seen['odom'] += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        seen['offsets'].append(time.time() - stamp)

    def on_scan(_):
        seen['scan'] += 1

    def on_battery(msg):
        seen['battery'] = float(msg.data)

    node.create_subscription(Odometry, 'odom', on_odom, 50)
    node.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(Float32, 'battery/voltage', on_battery, 1)
    domain = node.context.get_domain_id() if hasattr(node.context, 'get_domain_id') else '?'
    print(f'ROS_DOMAIN_ID={domain} 에서 {args.seconds:.0f}초 동안 듣습니다...')
    time.sleep(1.0)                           # 서로 찾는 시간
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    offsets = sorted(seen['offsets'])
    rows = judge(
        seen['odom'], seen['scan'], args.seconds,
        node.count_publishers('odom'), node.count_publishers('scan'),
        offsets[len(offsets) // 2] if offsets else None,       # 가운데 값 (와이파이 지연 튐 제외)
        'cmd_vel_watchdog' in node.get_node_names(),
        node.count_subscribers('cmd_vel'), seen['battery'], bridged=args.bridged)
    for mark, item, text in rows:
        print(f' {mark} {item}: {text}')
    bad = sum(1 for mark, _, _ in rows if mark == BAD)
    print('점검 통과' if not bad else f'문제 {bad}개: 고친 뒤 다시 돌리세요')
    node.destroy_node()
    rclpy.try_shutdown()
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
