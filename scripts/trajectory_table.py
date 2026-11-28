#!/usr/bin/env python3
"""
관제 서버가 남긴 궤적 파일을 표로 뽑는다 (파라미터 튜닝 로그용). ROS 없이 돈다.

  python3 scripts/trajectory_table.py logs/trajectories
  python3 scripts/trajectory_table.py logs/trajectories --action park --real --since 20261125
  python3 scripts/trajectory_table.py logs/trajectories --csv park.csv --markdown

작업마다 한 줄: 시각, 로봇, 행동, 대상, 실물 여부, 차종, 주행 설정(속도, 회전, 주시 거리, 도착
허용 오차), 상태, 걸린 시간, 경로 오차(이어진 경로를 따라간 작업만), 도착 오차.
마지막에 같은 설정끼리 묶은 요약(횟수, 성공 수, 평균 시간, 평균 오차)을 낸다. 평균에는 성공하고
실제로 움직인 작업만 넣는다(이미 칸 안에 있어 바로 끝난 주차 등은 뺀다).

- 경로 오차와 도착 오차는 로봇이 스스로 계산한 위치(AMCL 또는 바퀴 기록) 기준이다. 실제 바닥
  위치와의 차이는 줄자로 따로 잰다 (scripts/park_accuracy.py --manual).
- 도착 오차: 마지막 목표점과 기록된 마지막 위치의 거리 [mm]와 방향 차이 [°]. 위치 기록(amcl_pose)은
  로봇이 일정 거리를 움직여야 새로 나와서, 마지막 기록이 작업 끝보다 1초 넘게 앞서면 비워 둔다.
- 주행 설정을 남기기 전에 만든 옛 파일은 설정 칸이 빈다.
- 파일 이름의 시각은 관제 서버를 돌린 곳의 시계다. Docker 컨테이너는 UTC 라서 한국 시각보다 9시간
  이르다. --since 에는 파일 이름의 시각을 쓴다.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys

COLUMNS = ['time', 'robot', 'action', 'target', 'real', 'vehicle',
           'speed', 'turn', 'lookahead', 'goal_tolerance', 'state', 'elapsed_s',
           'path_mean_mm', 'path_max_mm', 'end_mm', 'end_deg', 'file']
# 요약에서 같은 조건으로 묶는 칸
GROUP = ['action', 'real', 'vehicle', 'speed', 'turn', 'lookahead', 'goal_tolerance']


INSTANT_S = 1.0   # 이보다 빨리 끝난 작업은 움직이지 않은 것으로 보고 평균에서 뺀다 [s]
STALE_S = 1.0     # 마지막 위치 기록이 작업 끝보다 이만큼 넘게 앞서면 도착 오차를 내지 않는다


def wrap(angle):
    """각도를 -π ~ π 로."""
    return math.atan2(math.sin(angle), math.cos(angle))


def end_error(planned, actual, elapsed_s=None):
    """
    마지막 목표점과 마지막 기록 위치의 차이 (거리 mm, 방향 °).

    기록이 없거나, 마지막 기록이 작업 끝보다 STALE_S 넘게 앞서면 (None, None). 위치 기록은
    amcl_pose 라서 로봇이 일정 거리를 움직여야 새로 나오고, 마지막 몇 cm 는 빠질 수 있다.
    """
    if not planned or not actual:
        return None, None
    goal, last = planned[-1], actual[-1]           # actual 한 줄: [t, x, y, yaw, v, w]
    if elapsed_s is not None and elapsed_s - last[0] > STALE_S:
        return None, None
    dist = math.hypot(last[1] - goal['x'], last[2] - goal['y']) * 1000
    turn = math.degrees(wrap(last[3] - goal['yaw']))
    return round(dist, 1), round(turn, 2)


def row(path, data):
    """궤적 파일 하나 → 표 한 줄 (dict)."""
    settings = data.get('settings') or {}
    deviation = data.get('deviation') or {}
    end_mm, end_deg = end_error(data.get('planned'), data.get('actual'), data.get('elapsed_s'))
    stamp = path.name[:15]                          # 20261121-143615_pinky2_005_unpark.json
    real = data.get('real')

    def mm(key):
        value = deviation.get(key)
        return None if value is None else round(value * 1000, 1)

    return {
        'time': stamp, 'robot': data.get('robot'), 'action': data.get('action'),
        'target': data.get('target'),
        'real': '' if real is None else ('실물' if real else '시뮬'),
        'vehicle': data.get('vehicle') or '',
        'speed': settings.get('speed'), 'turn': settings.get('turn'),
        'lookahead': settings.get('lookahead'), 'goal_tolerance': settings.get('goal_tolerance'),
        'state': data.get('state'), 'elapsed_s': data.get('elapsed_s'),
        'path_mean_mm': mm('mean'), 'path_max_mm': mm('max'),
        'end_mm': end_mm, 'end_deg': end_deg, 'file': path.name,
        # 1초 안에 끝난 작업은 움직이지 않은 것이다 (이미 그 자리). 위치 기록이 한두 줄 남기도 한다
        'moved': bool(data.get('actual')) and (data.get('elapsed_s') or 0) >= INSTANT_S,
    }


def load(directory, action='', robot='', since='', real=None):
    """폴더의 궤적 파일을 시각 순으로 읽어 거른 줄 목록. 읽지 못한 파일은 건너뛰고 알린다."""
    rows = []
    for path in sorted(Path(directory).glob('*.json')):
        if since and path.name[:len(since)] < since:
            continue
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as error:
            print(f'읽지 못했습니다: {path.name} ({error})', file=sys.stderr)
            continue
        if action and data.get('action') != action:
            continue
        if robot and data.get('robot') != robot:
            continue
        if real is not None and data.get('real') is not real:
            continue
        rows.append(row(path, data))
    return rows


def mean(values):
    """None 을 뺀 평균 (소수 한 자리). 값이 없으면 None."""
    values = [v for v in values if v is not None]
    return round(statistics.mean(values), 1) if values else None


def summary(rows):
    """
    같은 조건(GROUP)끼리 묶어: 횟수, 성공 수, 성공한 작업의 평균 시간과 평균 오차.

    움직이지 않고 끝난 작업(이미 그 자리에 있었음)은 횟수에만 세고 평균에서는 뺀다.
    """
    groups = {}
    for r in rows:
        groups.setdefault(tuple(r[k] for k in GROUP), []).append(r)
    out = []
    for key, items in groups.items():
        succeeded = [r for r in items if r['state'] == 'succeeded']
        ok = [r for r in succeeded if r['moved']]
        out.append({**dict(zip(GROUP, key)), 'runs': len(items), 'succeeded': len(succeeded),
                    'elapsed_s': mean(r['elapsed_s'] for r in ok),
                    'path_mean_mm': mean(r['path_mean_mm'] for r in ok),
                    'end_mm': mean(r['end_mm'] for r in ok),
                    'end_deg': mean(None if r['end_deg'] is None else abs(r['end_deg'])
                                    for r in ok)})
    return out


def text(value):
    """표 칸 글자: None 은 빈 칸."""
    return '' if value is None else str(value)


def print_table(rows, columns, markdown):
    """줄 목록을 칸 맞춘 글자 표 또는 마크다운 표로 출력한다."""
    if markdown:
        print('| ' + ' | '.join(columns) + ' |')
        print('|' + '---|' * len(columns))
        for r in rows:
            print('| ' + ' | '.join(text(r[c]) for c in columns) + ' |')
        return
    widths = {c: max([len(c)] + [len(text(r[c])) for r in rows]) for c in columns}
    print('  '.join(c.ljust(widths[c]) for c in columns))
    for r in rows:
        print('  '.join(text(r[c]).ljust(widths[c]) for c in columns))


def main():
    """명령줄 인자를 읽어 표와 요약을 낸다. 맞는 파일이 없으면 1."""
    parser = argparse.ArgumentParser(description='궤적 파일을 표로 (파라미터 튜닝 로그용)')
    parser.add_argument('directory', help='궤적 폴더 (관제 서버의 <log-dir>/trajectories)')
    parser.add_argument('--action', default='', help='행동만: park, unpark, drive, lane_change 등')
    parser.add_argument('--robot', default='', help='로봇만: pinky1, pinky2')
    parser.add_argument('--since', default='', help='이 시각 이후 (파일 이름 앞부분, 예: 20261125-13)')
    kind = parser.add_mutually_exclusive_group()
    kind.add_argument('--real', dest='real', action='store_const', const=True, help='실물만')
    kind.add_argument('--sim', dest='real', action='store_const', const=False, help='시뮬레이션만')
    parser.add_argument('--csv', default='', help='작업별 표를 CSV 로 저장')
    parser.add_argument('--markdown', action='store_true', help='문서에 붙일 마크다운 표로 출력')
    parser.add_argument('--no-rows', action='store_true', help='작업별 표 없이 요약만')
    args = parser.parse_args()

    if not Path(args.directory).is_dir():
        parser.error(f'폴더가 없습니다: {args.directory}')
    rows = load(args.directory, args.action, args.robot, args.since, args.real)
    if not rows:
        print('조건에 맞는 궤적 파일이 없습니다')
        return 1
    if args.csv:
        with open(args.csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=COLUMNS, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(rows)
        print(f'CSV 저장: {args.csv} ({len(rows)}줄)', file=sys.stderr)
    if not args.no_rows:
        print_table(rows, COLUMNS[:-1], args.markdown)
        print()
    groups = summary(rows)
    print(f'요약 (같은 설정끼리, 시간·오차는 성공한 작업만, 작업 {len(rows)}개)')
    print_table(groups, GROUP + ['runs', 'succeeded', 'elapsed_s', 'path_mean_mm',
                                 'end_mm', 'end_deg'], args.markdown)
    return 0


if __name__ == '__main__':
    sys.exit(main())
