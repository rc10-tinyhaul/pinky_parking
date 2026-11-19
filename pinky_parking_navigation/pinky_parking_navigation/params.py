"""로봇별 Nav2 파라미터."""
import copy


def nav2_params(base_params, x, y, yaw, footprint=None):
    """
    공식 nav2_params.yaml 에서 필요한 값만 바꾼 사본을 돌려준다.

    - AMCL 초기 위치: 공식 파일의 initial_pose 는 목록([0, 0, 0])인데 Jazzy AMCL 은
      initial_pose.x/.y/.z/.yaw 로 읽는다.
    - footprint ([[x, y], ...]): 주면 local·global costmap 의 footprint 를 바꾼다(트레일러 차량).
      None 이면 공식 값을 그대로 둔다.
    """
    params = copy.deepcopy(base_params)
    amcl = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
    if footprint is not None:
        text = '[' + ', '.join(f'[{px:g}, {py:g}]' for px, py in footprint) + ']'
        for costmap in ('local_costmap', 'global_costmap'):
            node = params.setdefault(costmap, {}).setdefault(costmap, {})
            node.setdefault('ros__parameters', {})['footprint'] = text
    return params
