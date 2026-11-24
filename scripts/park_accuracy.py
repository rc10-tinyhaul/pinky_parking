#!/usr/bin/env python3
"""
주차 정지 오차를 반복해서 잰다.

관제 API 로 출차 → 주차를 되풀이하고, 주차할 때마다 로봇의 실제 자리를 칸 가운데와 비교한다.

  # 시뮬레이션 (컨테이너 안, 시뮬레이션과 관제 서버가 떠 있어야 한다): 실제 자리는 Gazebo 가 안다
  python3 scripts/park_accuracy.py --robot pinky2 --slot slot_10 --runs 10 --csv park.csv

  # 실물: 실제 자리를 줄자로 재서 입력한다. 출차·주차 명령 전에 Enter 를 기다린다
  python3 scripts/park_accuracy.py --manual --robot pinky1 --slot slot_11 --runs 10 --csv park.csv

결과: 회차마다 칸 가운데에서 옆(칸 폭 방향)·앞뒤(칸 길이 방향) 어긋남 [mm], 방향 어긋남 [°], 걸린 시간.
마지막에 거리·방향 오차의 평균과 최대. 방향은 칸 방향 또는 그 반대 중 가까운 쪽과 비교한다.

부호 (줄자 입력도 같다): 앞뒤는 칸 방향(arena.yaml 의 yaw) 쪽이 +, 옆은 칸 방향에서 왼쪽이 +,
방향은 위에서 보아 반시계가 +. CSV 머리말(# 줄)에는 그때의 주행 설정(관제 화면 값)이 남는다.

로봇이 믿는 자리는 남기지 않는다. 관제 API 의 위치는 마지막 amcl_pose 라서 멈춘 뒤에는 새로
나오지 않아, 마지막 몇 cm 를 놓친다 (시뮬레이션에서 실제 18 mm 일 때 157 mm 로 나옴).
관제 서버에 팀 비밀번호가 켜져 있으면 환경변수 PINKY_CONTROL_PASSWORD 로 준다. Ctrl+C 로 멈춰도
그때까지 잰 회차는 CSV 에 남는다.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
ARENA_YAML = ROOT / 'pinky_parking_control/config/arena.yaml'
COLUMNS = ['run', 'side_mm', 'along_mm', 'yaw_deg', 'park_s', 'state']


def api(base, method, path, body=None):
    """관제 API 호출. 응답 JSON 을 돌려준다."""
    data = json.dumps(body).encode() if body is not None else None
    headers = {'Content-Type': 'application/json'}
    if os.environ.get('PINKY_CONTROL_PASSWORD'):         # 관제 서버에 팀 비밀번호가 켜져 있을 때
        headers['X-Control-Password'] = os.environ['PINKY_CONTROL_PASSWORD']
    request = urllib.request.Request(base + path, data=data, method=method, headers=headers)
    with urllib.request.urlopen(request, timeout=10) as reply:
        return json.loads(reply.read() or b'null')


def robot_status(base, robot):
    """관제 API 의 그 로봇 상태."""
    return next(r for r in api(base, 'GET', '/api/robots') if r['name'] == robot)


def wait_task(base, robot, timeout):
    """그 로봇의 작업이 끝날 때까지 기다려 (상태, 설명, 걸린 시간)."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        task = robot_status(base, robot)['task']
        if task and task['state'] != 'running':
            return task['state'], task.get('detail', ''), task.get('elapsed_s')
        time.sleep(1.0)
    return 'timeout', '', None


def gazebo_pose(model):
    """Gazebo 모델의 실제 자리 (x, y, yaw). gz CLI 의 출력을 읽는다."""
    out = subprocess.run(['gz', 'model', '-m', model, '-p'], capture_output=True, text=True,
                         timeout=10).stdout
    numbers = [n for n in re.findall(r'\[([-\d.e ]+)\]', out) if len(n.split()) == 3]
    if len(numbers) < 2:
        raise RuntimeError(f'Gazebo 에서 {model} 의 자리를 읽지 못했습니다:\n{out}')
    x, y, _ = (float(v) for v in numbers[0].split())
    yaw = float(numbers[1].split()[2])
    return x, y, yaw


def slot_error(pose, slot):
    """칸 기준 어긋남: (옆 [m], 앞뒤 [m], 방향 [rad]). 방향은 칸 방향과 그 반대 중 가까운 쪽."""
    dx, dy = pose[0] - slot['x'], pose[1] - slot['y']
    c, s = math.cos(slot['yaw']), math.sin(slot['yaw'])
    along, side = dx * c + dy * s, -dx * s + dy * c
    turn = math.atan2(math.sin(pose[2] - slot['yaw']), math.cos(pose[2] - slot['yaw']))
    if abs(turn) > math.pi / 2:
        turn = math.atan2(math.sin(turn + math.pi), math.cos(turn + math.pi))
    return side, along, turn


def parse_measurement(text):
    """
    줄자 값 한 줄 → (옆 [m], 앞뒤 [m], 방향 [rad]). 빈 줄은 None (이 회차를 건너뜀).

    '옆 앞뒤 방향' 을 mm, mm, 도 로 띄어쓰기나 쉼표로 적는다 (예: '12 -5 2.5'). 형식이 틀리면 ValueError.
    """
    parts = [p for p in re.split(r'[\s,]+', text.strip()) if p]
    if not parts:
        return None
    if len(parts) != 3:
        raise ValueError('옆 mm, 앞뒤 mm, 방향 ° 세 값을 적습니다 (예: 12 -5 2.5)')
    try:
        side, along, turn = (float(p) for p in parts)
    except ValueError:
        raise ValueError(f'숫자가 아닙니다: {text.strip()}')
    return side / 1000, along / 1000, math.radians(turn)


def ask_measurement(prompt=input):
    """줄자 값을 맞게 적을 때까지 묻는다."""
    while True:
        try:
            return parse_measurement(prompt('  줄자 값 (옆 mm, 앞뒤 mm, 방향 °; 건너뛰려면 빈 줄): '))
        except ValueError as error:
            print(f'  {error}')


def confirm(text, prompt=input):
    """사람 확인. q 를 적으면 False (측정을 끝낸다)."""
    return prompt(f'  {text} [Enter 실행, q 끝] ').strip().lower() != 'q'


def summary(rows):
    """[(옆, 앞뒤, 방향)] → 거리·방향 오차의 평균과 최대 (mm, 도)."""
    dist = [math.hypot(r[0], r[1]) * 1000 for r in rows]
    turn = [abs(math.degrees(r[2])) for r in rows]
    return {'n': len(rows), 'dist_mean': sum(dist) / len(dist), 'dist_max': max(dist),
            'yaw_mean': sum(turn) / len(turn), 'yaw_max': max(turn)}


def describe(name, s):
    """요약 한 줄."""
    return (f'{name} {s["n"]}회: 거리 오차 평균 {s["dist_mean"]:.1f} mm, 최대 {s["dist_max"]:.1f} mm'
            f' / 방향 오차 평균 {s["yaw_mean"]:.2f}°, 최대 {s["yaw_max"]:.2f}°')


def fmt(error):
    """(옆, 앞뒤, 방향) → CSV 칸 세 개. 없으면 빈 칸."""
    if error is None:
        return ['', '', '']
    return [f'{error[0] * 1000:.1f}', f'{error[1] * 1000:.1f}', f'{math.degrees(error[2]):.2f}']


def settings_header(base):
    """CSV 머리말: 그때의 주행 설정 (관제 API /api/tuning). 못 읽으면 그렇다고 적는다."""
    try:
        values = {item['key']: item['value'] for item in api(base, 'GET', '/api/tuning')}
    except (OSError, ValueError, KeyError, TypeError) as error:
        return [f'# settings: 읽지 못함 ({error})']
    return ['# settings (관제 화면 값. 주차 진입의 도착 허용 오차는 관제가 3 cm 로 좁힌다): '
            + ', '.join(f'{k}={v}' for k, v in values.items())]


def main():
    """명령줄 인자대로 출차 → 주차를 되풀이하며 정지 오차를 잰다."""
    parser = argparse.ArgumentParser(description='주차 정지 오차 반복 측정')
    parser.add_argument('--robot', default='pinky2')
    parser.add_argument('--slot', default='slot_10')
    parser.add_argument('--runs', type=int, default=10)
    parser.add_argument('--api', default='http://localhost:8080')
    parser.add_argument('--timeout', type=float, default=180.0, help='작업마다 [s]')
    parser.add_argument('--csv', default='', help='회차별 결과를 저장할 CSV')
    parser.add_argument('--manual', action='store_true',
                        help='실물: 실제 자리를 줄자로 재서 입력. 출차·주차 전에 사람 확인을 받는다')
    args = parser.parse_args()

    import yaml
    slot = yaml.safe_load(ARENA_YAML.read_text(encoding='utf-8'))['slots'][args.slot]
    rows = []
    lines = [f'# robot={args.robot}, slot={args.slot}, '
             f'truth={"tape" if args.manual else "gazebo"}, '
             f'started={time.strftime("%Y-%m-%d %H:%M:%S")}',
             *settings_header(args.api), ','.join(COLUMNS)]
    print(f'{args.robot} → {args.slot} ({slot["x"]}, {slot["y"]}) {args.runs}회')
    if args.manual:
        print(f'부호: 앞뒤 = 칸 방향(yaw {math.degrees(slot["yaw"]):.0f}°) 쪽 +, 옆 = 그 왼쪽 +, '
              '방향 = 반시계 +. 로봇 바퀴 축 가운데를 잽니다.')
    try:
        for run in range(1, args.runs + 1):
            if args.manual and not confirm(f'{run}회: 출차할까요?'):
                break
            api(args.api, 'POST', f'/api/robots/{args.robot}/unpark', {'slot': args.slot})
            state, detail, _ = wait_task(args.api, args.robot, args.timeout)
            if state != 'succeeded':
                print(f'  {run}: 출차 {state} {detail}')
                break
            if args.manual and not confirm(f'{run}회: 주차할까요?'):
                break
            api(args.api, 'POST', f'/api/robots/{args.robot}/park', {'slot': args.slot})
            state, detail, seconds = wait_task(args.api, args.robot, args.timeout)
            time.sleep(1.0)                                     # 멈춘 뒤 자리를 읽는다
            truth = (ask_measurement() if args.manual
                     else slot_error(gazebo_pose(args.robot), slot))
            lines.append(','.join([str(run), *fmt(truth), str(seconds), state]))
            shown = ', '.join(f'{name} {v}' for name, v in zip(
                ('옆', '앞뒤', '방향'), fmt(truth)) if v) or '실제 자리 없음'
            print(f'  {run:2d}: {shown}, {seconds} s, {state} {detail}')
            if state == 'succeeded' and truth is not None:
                rows.append(truth)
    except KeyboardInterrupt:
        print('\n멈췄습니다. 지금까지 잰 회차를 남깁니다.')
    if args.csv:
        Path(args.csv).write_text('\n'.join(lines) + '\n', encoding='utf-8')
        print(f'CSV 저장: {args.csv}')
    if not rows:
        print('실제 자리를 잰 성공한 주차가 없습니다')
        sys.exit(1)
    print(describe('실제 자리', summary(rows)))


if __name__ == '__main__':
    main()
