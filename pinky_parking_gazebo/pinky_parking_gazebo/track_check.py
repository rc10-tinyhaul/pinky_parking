"""
차선 시험 트랙에서 로봇이 차선 중심선을 얼마나 잘 따라가는지 잰다 (Gazebo 의 실제 위치 기준).

  ros2 run pinky_parking_gazebo track_check pinky1 --laps 3

Gazebo 위치는 gz CLI 로 읽으므로 ROS_DOMAIN_ID 와 무관하다. 반시계 방향 바퀴 수를 세고,
중심선에서 벗어난 거리의 평균·최대와 차체가 테이프에 닿은 시간 비율을 출력한다.
정한 바퀴 수를 다 돌면 종료 코드 0, 시간 초과나 차선 이탈이면 1.
"""
import argparse
import json
import math
import subprocess
import sys
import time

from pinky_parking_gazebo.lane_track import lane_offset
from pinky_parking_gazebo.lane_track import load_track

BODY_HALF_WIDTH = 0.065   # Pinky Pro 폭 130 mm 의 절반 [m]


def poses(world, name):
    """Gazebo 에서 모델 위치를 (x, y, 시뮬레이션 시각 [s]) 로 계속 읽는다."""
    process = subprocess.Popen(
        ['gz', 'topic', '-e', '-t', f'/world/{world}/dynamic_pose/info', '--json-output'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    try:
        for line in process.stdout:
            try:
                message = json.loads(line)
            except ValueError:
                continue
            stamp = message.get('header', {}).get('stamp', {})
            sim_time = float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0)) / 1e9
            for pose in message.get('pose', []):
                if pose.get('name') == name:
                    position = pose.get('position', {})
                    yield position.get('x', 0.0), position.get('y', 0.0), sim_time
                    break
    finally:
        process.kill()


def main():
    parser = argparse.ArgumentParser(description='차선 시험 트랙 주행 채점')
    parser.add_argument('name', help='로봇 이름 (Gazebo 모델 이름, 예: pinky1)')
    parser.add_argument('--laps', type=float, default=1.0, help='잴 바퀴 수')
    parser.add_argument('--lane', type=int, help='차선 번호 (기본: 출발 때 가장 가까운 차선)')
    parser.add_argument('--timeout', type=float, default=180.0, help='제한 시간 [s, 실제 시간]')
    parser.add_argument('--track', help='트랙 치수 파일 (기본: config/lane_track.yaml)')
    parser.add_argument('--world', default='default', help='Gazebo 월드 이름')
    args = parser.parse_args()

    track = load_track(args.track)
    margin = (track['lane_width'] - track['tape_width']) / 2   # 중심에서 테이프 안쪽 가장자리까지
    lane, angle, start_angle = args.lane, None, None
    offsets, touching, start = [], 0, time.time()
    for x, y, sim_time in poses(args.world, args.name):
        lane, offset = lane_offset(track, x, y, lane)
        a = math.atan2(y, x)
        if angle is None:
            angle = start_angle = a
            sim_start = sim_time
        angle += (a - angle + math.pi) % (2 * math.pi) - math.pi   # 이어지는 각도
        offsets.append(abs(offset))
        touching += abs(offset) + BODY_HALF_WIDTH > margin
        laps = (angle - start_angle) / (2 * math.pi)
        if abs(offset) > margin + track['tape_width']:
            print(f'차선 이탈: 중심에서 {offset * 1000:+.0f} mm')
            break
        if laps >= args.laps or time.time() - start > args.timeout:
            break
    else:
        print('Gazebo 위치를 읽지 못했습니다')
        return 1

    done = laps >= args.laps
    mean, worst = sum(offsets) / len(offsets) * 1000, max(offsets) * 1000
    wall, sim = time.time() - start, sim_time - sim_start
    print(f'{args.name} {lane}번 차선: {laps:.2f}바퀴, {wall:.0f}s '
          f'(시뮬레이션 {sim:.0f}s, 실시간의 {sim / wall:.2f}배), '
          f'중심에서 평균 {mean:.0f} mm / 최대 {worst:.0f} mm, '
          f'차체가 테이프에 닿은 비율 {touching / len(offsets) * 100:.0f}% '
          f'(여유 {(margin - BODY_HALF_WIDTH) * 1000:.0f} mm)')
    print('통과' if done else '실패')
    return 0 if done else 1


if __name__ == '__main__':
    sys.exit(main())
