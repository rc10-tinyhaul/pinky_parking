"""
차선 시험 트랙에서 로봇이 차선 중심선을 얼마나 잘 따라가는지 잰다 (Gazebo 의 실제 위치 기준).

  ros2 run pinky_parking_gazebo track_check pinky1 --laps 3
  ros2 run pinky_parking_gazebo track_check pinky1 --laps 3 --change 2   (차선 변경 2번 포함)

Gazebo 위치는 gz CLI 로 읽으므로 ROS_DOMAIN_ID 와 무관하다. 반시계 방향 바퀴 수를 세고,
중심선에서 벗어난 거리의 평균·최대와 차체가 테이프에 닿은 시간 비율을 출력한다.
--change 를 주면 차선 변경마다 시작·끝 위치(직선 안인지)와 넘어간 거리를 출력하고,
변경 구간은 평균·최대 계산에서 뺀다.
정한 바퀴 수를 다 돌면 종료 코드 0. 시간 초과, 이탈, 변경 횟수가 다르거나 직선 밖 변경이면 1.
"""
import argparse
import json
import math
import subprocess
import sys
import time

from pinky_parking_gazebo.lane_track import core_distance
from pinky_parking_gazebo.lane_track import lane_offset
from pinky_parking_gazebo.lane_track import load_track
from pinky_parking_gazebo.lane_track import tape_radii

BODY_HALF_WIDTH = 0.065   # Pinky Pro 폭 130 mm 의 절반 [m]
SETTLED = 0.02            # 차선 중심에서 이 안이면 "차선 안에 있음" [m]
MIN_SEGMENT = 0.05        # 이보다 짧게 다른 차선에 걸친 것은 변경으로 보지 않음 [m]
OVERSHOOT_WINDOW = 0.3    # 변경이 끝난 뒤 넘어간 거리를 보는 구간 [m]


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


def find_changes(track, samples):
    """
    차선 변경을 찾는다. samples: [(x, y, 누적 거리)].

    각 변경은 dict: from, to, start_y, end_y (트랙 좌표 y. 직선은 |y| <= 직선 길이/2),
    on_straight, overshoot [m], span (변경 구간 samples 인덱스 범위).
    """
    lanes = [lane_offset(track, x, y)[0] for x, y, _ in samples]
    # 같은 차선이 이어지는 구간으로 나누고, 아주 짧은 구간은 앞 구간에 붙인다
    segments = []
    for i, lane in enumerate(lanes):
        if segments and segments[-1][0] == lane:
            segments[-1][2] = i
        else:
            segments.append([lane, i, i])
    merged = []
    for seg in segments:
        short = samples[seg[2]][2] - samples[seg[1]][2] < MIN_SEGMENT
        if merged and (short or merged[-1][0] == seg[0]):
            merged[-1][2] = seg[2]
        else:
            merged.append(seg)

    half = track['straight_length'] / 2
    changes = []
    for before, after in zip(merged, merged[1:]):
        old, new, cross = before[0], after[0], after[1]
        start = next((i for i in range(cross - 1, before[1] - 1, -1)
                      if abs(lane_offset(track, *samples[i][:2], old)[1]) <= SETTLED), before[1])
        end = next((i for i in range(cross, after[2] + 1)
                    if abs(lane_offset(track, *samples[i][:2], new)[1]) <= SETTLED), after[2])
        sign = 1 if new > old else -1      # 바깥 차선으로 가면 + 쪽으로 넘어감
        window = [i for i in range(end, after[2] + 1)
                  if samples[i][2] - samples[end][2] <= OVERSHOOT_WINDOW]
        overshoot = max([0.0] + [sign * lane_offset(track, *samples[i][:2], new)[1]
                                 for i in window])
        start_y, end_y = samples[start][1], samples[end][1]
        changes.append({'from': old, 'to': new, 'start_y': start_y, 'end_y': end_y,
                        'on_straight': abs(start_y) <= half and abs(end_y) <= half,
                        'overshoot': overshoot, 'span': (start, end)})
    return changes


def main():
    parser = argparse.ArgumentParser(description='차선 시험 트랙 주행 채점')
    parser.add_argument('name', help='로봇 이름 (Gazebo 모델 이름, 예: pinky1)')
    parser.add_argument('--laps', type=float, default=1.0, help='잴 바퀴 수')
    parser.add_argument('--lane', type=int, help='차선 번호 (기본: 출발 때 가장 가까운 차선)')
    parser.add_argument('--change', type=int, help='차선 변경 횟수 (주면 변경을 허용하고 채점)')
    parser.add_argument('--timeout', type=float, default=180.0, help='제한 시간 [s, 실제 시간]')
    parser.add_argument('--track', help='트랙 치수 파일 (기본: config/lane_track.yaml)')
    parser.add_argument('--world', default='default', help='Gazebo 월드 이름')
    args = parser.parse_args()

    track = load_track(args.track)
    margin = (track['lane_width'] - track['tape_width']) / 2   # 중심에서 테이프 안쪽 가장자리까지
    radii = tape_radii(track)
    inside, outside = radii[0] - track['tape_width'] / 2, radii[-1] + track['tape_width'] / 2
    lane, angle, start_angle = args.lane, None, None
    samples, travelled, start = [], 0.0, time.time()
    for x, y, sim_time in poses(args.world, args.name):
        if args.change is None:
            lane, offset = lane_offset(track, x, y, lane)
        else:
            lane, offset = lane_offset(track, x, y)
        a = math.atan2(y, x)
        if angle is None:
            angle = start_angle = a
            sim_start = sim_time
        else:
            travelled += math.hypot(x - samples[-1][0], y - samples[-1][1])
        angle += (a - angle + math.pi) % (2 * math.pi) - math.pi   # 이어지는 각도
        samples.append((x, y, travelled))
        laps = (angle - start_angle) / (2 * math.pi)
        distance = core_distance(track, x, y)
        departed = (abs(offset) > margin + track['tape_width'] if args.change is None
                    else not inside <= distance <= outside)
        if departed:
            print(f'차선 이탈: {lane}번 차선 중심에서 {offset * 1000:+.0f} mm')
            break
        if laps >= args.laps or time.time() - start > args.timeout:
            break
    else:
        print('Gazebo 위치를 읽지 못했습니다')
        return 1

    changes = find_changes(track, samples) if args.change is not None else []
    in_change = set()
    for change in changes:
        in_change.update(range(change['span'][0], change['span'][1] + 1))
    offsets = [abs(lane_offset(track, x, y, None if args.change is not None else lane)[1])
               for i, (x, y, _) in enumerate(samples) if i not in in_change]
    touching = sum(o + BODY_HALF_WIDTH > margin for o in offsets)

    done = laps >= args.laps and not departed
    mean, worst = sum(offsets) / len(offsets) * 1000, max(offsets) * 1000
    wall, sim = time.time() - start, sim_time - sim_start
    print(f'{args.name} {lane}번 차선: {laps:.2f}바퀴, {wall:.0f}s '
          f'(시뮬레이션 {sim:.0f}s, 실시간의 {sim / wall:.2f}배), '
          f'중심에서 평균 {mean:.0f} mm / 최대 {worst:.0f} mm, '
          f'차체가 테이프에 닿은 비율 {touching / len(offsets) * 100:.0f}% '
          f'(여유 {(margin - BODY_HALF_WIDTH) * 1000:.0f} mm)'
          + (', 변경 구간 제외' if changes else ''))
    for change in changes:
        print(f'  차선 변경 {change["from"]}→{change["to"]}: '
              f'y {change["start_y"]:+.2f} → {change["end_y"]:+.2f} m '
              f'({"직선 안" if change["on_straight"] else "직선 밖"}), '
              f'넘어간 거리 {change["overshoot"] * 1000:.0f} mm')
    if args.change is not None:
        if len(changes) != args.change:
            print(f'차선 변경 {len(changes)}번 (기대 {args.change}번)')
            done = False
        if not all(change['on_straight'] for change in changes):
            done = False
    print('통과' if done else '실패')
    return 0 if done else 1


if __name__ == '__main__':
    sys.exit(main())
