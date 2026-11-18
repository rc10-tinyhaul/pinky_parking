"""로봇별 Nav2 파라미터."""
import copy


def nav2_params(base_params, x, y, yaw):
    """
    공식 nav2_params.yaml 에서 AMCL 초기 위치만 바꾼 사본을 돌려준다.

    공식 파일의 initial_pose 는 목록([0, 0, 0])인데 Jazzy AMCL 은
    initial_pose.x/.y/.z/.yaw 로 읽는다.
    """
    params = copy.deepcopy(base_params)
    amcl = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
    return params
