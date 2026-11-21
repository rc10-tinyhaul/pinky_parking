"""로봇별 Nav2 파라미터."""
import copy


def nav2_params(base_params, x, y, yaw, max_speed=None):
    """
    공식 nav2_params.yaml 에서 필요한 값만 바꾼 사본을 돌려준다.

    - AMCL 초기 위치: 공식 파일의 initial_pose 는 목록([0, 0, 0])인데 Jazzy AMCL 은
      initial_pose.x/.y/.z/.yaw 로 읽는다.
    - max_speed [m/s]: 주면 직진 속도 상한을 낮춘다(실물 첫 시험용). 컨트롤러의 목표 속도와
      속도 평활기의 최대 속도를 함께 낮춘다. 공식 값보다 높이지는 않는다.
    """
    params = copy.deepcopy(base_params)
    amcl = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
    if max_speed is not None:
        controller = params.setdefault('controller_server', {}).setdefault('ros__parameters', {})
        follow = controller.setdefault('FollowPath', {})
        follow['desired_linear_vel'] = min(follow.get('desired_linear_vel', max_speed), max_speed)
        smoother = params.setdefault('velocity_smoother', {}).setdefault('ros__parameters', {})
        top = smoother.get('max_velocity', [max_speed, 0.0, 1.5])
        bottom = smoother.get('min_velocity', [-max_speed, 0.0, -1.5])
        smoother['max_velocity'] = [min(top[0], max_speed)] + list(top[1:])
        smoother['min_velocity'] = [max(bottom[0], -max_speed)] + list(bottom[1:])
    return params
