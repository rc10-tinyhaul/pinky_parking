"""
주행 설정: 관제 화면에서 달리는 중에 바꾸는 값 (속도, 회전, 주시 거리, 도착 허용 거리, 직접 운전 감도).

값 하나는 Nav2 노드의 파라미터 몇 개로 옮겨진다. ROS 없이 시험할 수 있다.
"""

# 키: (이름, 단위, 최소, 최대, 눈금, 기본값, 설명)
SETTINGS = {
    'speed': ('주행 속도', 'm/s', 0.05, 0.25, 0.01, 0.20, '트랙·이동 때 직진 속도'),
    'turn': ('회전 속도', 'rad/s', 0.3, 1.5, 0.1, 0.8, '제자리 회전과 곡선에서의 회전 상한'),
    'lookahead': ('곡선 감도', 'm', 0.10, 0.50, 0.02, 0.20,
                  '경로를 내다보는 거리. 짧으면 경로에 바짝 붙고(흔들릴 수 있음), 길면 부드럽지만 안쪽으로 돈다'),
    'goal_tolerance': ('도착 허용 거리', 'm', 0.03, 0.25, 0.01, 0.08, '목표에서 이 거리 안이면 도착'),
    'teleop_linear': ('직접 운전 직진', 'm/s', 0.03, 0.25, 0.01, 0.15, '조이스틱·키보드 최대 직진 속도'),
    'teleop_angular': ('직접 운전 회전', 'rad/s', 0.3, 2.0, 0.1, 1.2, '조이스틱·키보드 최대 회전 속도'),
}
NAV2_KEYS = ('speed', 'turn', 'lookahead', 'goal_tolerance')


def describe(values):
    """화면에 보낼 목록: 범위와 지금 값."""
    return [{'key': key, 'name': name, 'unit': unit, 'min': low, 'max': high, 'step': step,
             'default': default, 'help': text, 'value': values.get(key, default)}
            for key, (name, unit, low, high, step, default, text) in SETTINGS.items()]


def clean(changes):
    """받은 값을 범위 안으로 자른다. 모르는 키나 숫자가 아닌 값은 ValueError."""
    out = {}
    for key, value in (changes or {}).items():
        if key not in SETTINGS:
            raise ValueError(f'모르는 설정입니다: {key}')
        try:
            number = float(value)
        except (TypeError, ValueError):
            raise ValueError(f'{SETTINGS[key][0]}: 숫자가 아닙니다')
        if number != number:
            raise ValueError(f'{SETTINGS[key][0]}: 숫자가 아닙니다')
        low, high = SETTINGS[key][2], SETTINGS[key][3]
        out[key] = round(max(low, min(high, number)), 3)
    return out


def nav2_parameters(values):
    """설정 값 → {Nav2 노드 이름: {파라미터: 값}}. 주어진 키만 바꾼다."""
    controller, smoother = {}, {}
    if 'speed' in values:
        controller['FollowPath.desired_linear_vel'] = values['speed']
    if 'turn' in values:
        controller['FollowPath.rotate_to_heading_angular_vel'] = values['turn']
    if 'lookahead' in values:
        controller['FollowPath.lookahead_dist'] = values['lookahead']
        controller['FollowPath.min_lookahead_dist'] = values['lookahead']
    if 'goal_tolerance' in values:
        controller['general_goal_checker.xy_goal_tolerance'] = values['goal_tolerance']
    if 'speed' in values and 'turn' in values:      # 속도 평활기의 상한은 둘을 함께 준다
        top = round(values['speed'] * 1.25, 3)
        smoother['max_velocity'] = [top, 0.0, values['turn']]
        smoother['min_velocity'] = [-top, 0.0, -values['turn']]
    out = {}
    if controller:
        out['controller_server'] = controller
    if smoother:
        out['velocity_smoother'] = smoother
    return out


# Nav2 에서 지금 값을 읽어 올 파라미터 (설정 키 → 노드, 파라미터)
NAV2_SOURCES = {
    'speed': ('controller_server', 'FollowPath.desired_linear_vel'),
    'turn': ('controller_server', 'FollowPath.rotate_to_heading_angular_vel'),
    'lookahead': ('controller_server', 'FollowPath.lookahead_dist'),
    'goal_tolerance': ('controller_server', 'general_goal_checker.xy_goal_tolerance'),
}
