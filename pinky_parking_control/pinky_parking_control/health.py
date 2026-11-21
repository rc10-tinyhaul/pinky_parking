"""
로봇 상태 알림: 왜 안 움직이는지, 무엇을 확인해야 하는지 사람이 읽을 문장으로 만든다.

- Nav2 가 낸 경고·오류 문장과 결과 코드를 우리말 사유로 옮긴다.
- 명령을 받았는데 움직이지 않는 것(멈춤)을 알아챈다.
- 로봇 한 대의 상태에서 화면에 띄울 알림 목록을 만든다.

ROS 를 import 하지 않는다.
"""
import math

ERROR, WARNING = 'error', 'warning'
BATTERY_EMPTY_V, BATTERY_LOW_V = 7.0, 7.3    # 공식 안내: 7 V 아래로 내려가면 충전이 안 될 수 있다
STALL_S = 4.0              # 작업 중인데 이 시간 동안 움직이지 않으면 '멈춤'
STALL_MOVE = 0.02          # 이만큼 [m] 은 움직여야 움직인 것
STALL_TURN = 0.08          # 또는 이만큼 [rad] 돌아야
REASON_FRESH_S = 6.0       # 이보다 오래된 Nav2 경고는 지금 상황의 사유로 쓰지 않는다

# Nav2 로그 문장의 일부 → 사유 (위에서부터 먼저 맞는 것)
LOG_REASONS = (
    ('collision ahead', '앞에 장애물이 있어 멈췄습니다 (라이다가 본 물체)'),
    ('Controller patience exceeded', '장애물 때문에 더 가지 못해 포기했습니다'),
    ('Failed to make progress', '제자리에서 나아가지 못했습니다 (끼임, 바퀴 헛돎)'),
    ('No valid trajectories', '갈 수 있는 길이 없습니다 (장애물에 둘러싸임)'),
    ('Failed to create a plan', '목표까지 경로를 찾지 못했습니다'),
    ('failed to create plan', '목표까지 경로를 찾지 못했습니다'),
    ('no valid path', '목표까지 경로를 찾지 못했습니다'),
    ('Start occupied', '로봇이 지금 장애물 안에 있는 것으로 보입니다 (위치가 틀렸을 수 있음)'),
    ('Goal occupied', '목표 자리에 장애물이 있습니다'),
    ('outside of the', '목표가 지도 밖에 있습니다'),
    ('extrapolation', '로봇의 위치 정보가 늦게 옵니다 (와이파이 지연, 시계 차이)'),
    ('Timed out waiting for transform', '로봇의 위치 정보(TF)가 오지 않습니다'),
    ('Could not transform', '로봇의 위치 정보(TF)를 읽지 못했습니다'),
    ('Resulting plan has 0 poses', '보낸 경로가 비어 있습니다'),
)

# Nav2 액션 결과의 error_code → 사유
CODE_REASONS = {
    101: '컨트롤러 설정 오류', 102: '로봇의 위치 정보(TF)를 읽지 못했습니다',
    103: '보낸 경로가 잘못됐습니다', 104: '장애물 때문에 더 가지 못해 포기했습니다',
    105: '제자리에서 나아가지 못했습니다 (끼임, 바퀴 헛돎)', 106: '갈 수 있는 길이 없습니다',
    201: '경로 계획 설정 오류', 202: '로봇의 위치 정보(TF)를 읽지 못했습니다',
    203: '로봇이 지도 밖에 있습니다', 204: '목표가 지도 밖에 있습니다',
    205: '로봇이 지금 장애물 안에 있는 것으로 보입니다', 206: '목표 자리에 장애물이 있습니다',
    207: '경로 계획 시간 초과', 208: '목표까지 경로를 찾지 못했습니다',
}


def reason_from_log(text):
    """Nav2 로그 문장 → 사유. 아는 문장이 아니면 빈 문자열."""
    lowered = text.lower()
    for needle, reason in LOG_REASONS:
        if needle.lower() in lowered:
            return reason
    return ''


def failure_detail(error_code, recent_reason):
    """작업이 실패했을 때 기록에 남길 사유."""
    reason = CODE_REASONS.get(int(error_code or 0), '') or recent_reason
    return f'Nav2 가 작업을 중단했습니다: {reason}' if reason else 'Nav2 가 작업을 중단했습니다'


class StallWatch:
    """작업 중인데 움직이지 않는 시간을 잰다. 작업이 바뀌면 다시 센다."""

    def __init__(self):
        self.task_id, self.anchor, self.since = None, None, 0.0

    def update(self, task_id, pose, now):
        """지금 작업과 위치를 알려 준다. 돌려준다: 움직이지 않은 시간 [s] (작업이 없으면 0)."""
        if task_id is None or pose is None:
            self.task_id = None
            return 0.0
        moved = self.anchor is not None and (
            math.hypot(pose['x'] - self.anchor['x'], pose['y'] - self.anchor['y']) > STALL_MOVE
            or abs(math.atan2(math.sin(pose['yaw'] - self.anchor['yaw']),
                              math.cos(pose['yaw'] - self.anchor['yaw']))) > STALL_TURN)
        if task_id != self.task_id or self.anchor is None or moved:
            self.task_id, self.anchor, self.since = task_id, dict(pose), now
        return now - self.since


def alerts(status, stalled_s=0.0, now=0.0):
    """로봇 한 대의 상태 → 알림 목록 [{'level', 'text'}] (급한 것부터)."""
    out = []
    task = status.get('task') or {}
    running = task.get('state') == 'running'
    issue = status.get('issue') or {}
    reason = issue.get('text', '') if issue.get('age_s', 99) <= REASON_FRESH_S else ''
    if status.get('odom_hz') == 0:
        out.append((ERROR, '로봇 응답 없음: 전원과 와이파이를 확인하세요'))
    battery = status.get('battery_v')
    if battery is not None and battery < BATTERY_EMPTY_V:
        out.append((ERROR, f'배터리 {battery:.2f} V: 지금 충전하세요 (곧 꺼집니다)'))
    if running and status.get('odom_hz') != 0 and stalled_s >= STALL_S:
        why = reason or ('Nav2 가 꺼져 있습니다' if not status.get('nav2_ready')
                         else '사유를 알 수 없습니다 (Nav2 로그 확인)')
        out.append((ERROR, f'명령을 받았지만 {stalled_s:.0f}초째 움직이지 않습니다: {why}'))
    if task.get('state') == 'failed' and now - (task.get('finished') or 0) < 120:
        out.append((ERROR, f'{task.get("target", "")} 실패: {task.get("detail") or "사유 없음"}'))
    if not status.get('nav2_ready'):
        out.append((WARNING, 'Nav2 가 켜지지 않았습니다: 명령을 받을 수 없습니다'))
    elif status.get('odom_hz') != 0 and not status.get('localized'):
        out.append((WARNING, '위치를 모릅니다 (위치 추정이 꺼졌거나 늦습니다)'))
    if status.get('pose_confirmed') is False:
        out.append((WARNING, '위치를 확인하지 않았습니다: 주차 칸에 놓고 위치 맞추기를 누르세요'))
    if battery is not None and BATTERY_EMPTY_V <= battery < BATTERY_LOW_V:
        out.append((WARNING, f'배터리 {battery:.2f} V: 곧 충전이 필요합니다'))
    if running and reason and stalled_s < STALL_S:
        out.append((WARNING, reason))
    return [{'level': level, 'text': text} for level, text in out]
