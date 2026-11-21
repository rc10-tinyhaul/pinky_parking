"""로봇별 Nav2 파라미터."""
import copy


def nav2_params(base_params, x, y, yaw, max_speed=None, goal_tolerance=None, max_turn=None,
                lookahead=None):
    """
    공식 nav2_params.yaml 에서 필요한 값만 바꾼 사본을 돌려준다.

    - AMCL 초기 위치: 공식 파일의 initial_pose 는 목록([0, 0, 0])인데 Jazzy AMCL 은
      initial_pose.x/.y/.z/.yaw 로 읽는다.
    - max_speed [m/s]: 주면 직진 속도 상한을 낮춘다(실물 첫 시험용). 컨트롤러의 목표 속도와
      속도 평활기의 최대 속도를 함께 낮춘다. 공식 값보다 높이지는 않는다.
    - goal_tolerance [m]: 주면 도착 허용 거리(공식 0.25 m)를 바꾼다. 경기장은 주차 칸이 190 x 300 mm,
      출차 거리가 0.4 m 라서 0.25 m 로는 칸에 들어가거나 나오기 전에 도착으로 친다.
    - max_turn [rad/s]: 주면 회전 속도 상한을 낮춘다. 바퀴 기록(odom)만으로 위치를 잡을 때
      빠른 제자리 회전에서 바퀴가 미끄러져 방향 오차가 쌓이는 것을 줄인다.
    - lookahead [m]: 주면 경로 추종의 주시 거리를 이 값으로 고정하고(공식은 0.3 ~ 0.9 m), 도착 전
      감속 구간도 같은 크기로 줄인다(공식 0.6 m). 경기장 트랙은 반경 0.4 m 라서 멀리 보면 곡선
      안쪽으로 파고들고, 0.4 m 짜리 짧은 이동은 처음부터 끝까지 감속 구간이 된다.
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
    if goal_tolerance is not None:
        controller = params.setdefault('controller_server', {}).setdefault('ros__parameters', {})
        checker = controller.setdefault('general_goal_checker', {})
        checker['xy_goal_tolerance'] = goal_tolerance
    if lookahead is not None:
        controller = params.setdefault('controller_server', {}).setdefault('ros__parameters', {})
        follow = controller.setdefault('FollowPath', {})
        follow['use_velocity_scaled_lookahead_dist'] = False
        follow['lookahead_dist'] = lookahead
        follow['min_lookahead_dist'] = lookahead
        follow['approach_velocity_scaling_dist'] = max(lookahead, 0.25)
    if max_turn is not None:
        controller = params.setdefault('controller_server', {}).setdefault('ros__parameters', {})
        follow = controller.setdefault('FollowPath', {})
        follow['rotate_to_heading_angular_vel'] = min(
            follow.get('rotate_to_heading_angular_vel', max_turn), max_turn)
        smoother = params.setdefault('velocity_smoother', {}).setdefault('ros__parameters', {})
        top = list(smoother.get('max_velocity', [0.25, 0.0, max_turn]))
        bottom = list(smoother.get('min_velocity', [-0.25, 0.0, -max_turn]))
        smoother['max_velocity'] = top[:2] + [min(top[2], max_turn)]
        smoother['min_velocity'] = bottom[:2] + [max(bottom[2], -max_turn)]
    return params


SLAM_FRAME = 'slam'    # slam_toolbox 가 쓰는 지도 프레임 (켠 자리가 원점). map 과는 고정 변환으로 잇는다


def slam_params(base_params):
    """
    공식 mapper_params.yaml 에서 '달리면서 위치 잡기'에 맞게 바꾼 사본을 돌려준다.

    - map_frame: slam_toolbox 는 켠 자리를 지도 원점으로 삼는다(시작 위치를 줄 수 없다). 그래서 자기
      프레임(slam)을 쓰게 하고, launch 가 map -> slam 고정 변환(= 시작 자리)을 내서
      map 좌표가 경기장 도면 좌표가 되게 한다.  map -> slam -> odom -> base_footprint
    - 공식 값은 0.5 m / 0.5 rad 움직여야 위치를 다시 맞춘다. 경기장은 2 m 크기라 그 사이에
      바퀴 기록 오차가 차선 폭만큼 쌓인다. 0.1 m / 0.15 rad 마다 맞추게 줄인다.
    - 한 번에 고치는 폭을 줄이고 약한 맞춤과 loop closing 을 끈다 (위치가 튀지 않게).
    """
    params = copy.deepcopy(base_params)
    slam = params.setdefault('slam_toolbox', {}).setdefault('ros__parameters', {})
    slam['mode'] = 'mapping'
    slam['map_frame'] = SLAM_FRAME
    slam['minimum_travel_distance'] = 0.1
    slam['minimum_travel_heading'] = 0.15
    slam['minimum_time_interval'] = 0.2
    slam['map_update_interval'] = 2.0
    # 사람이 오가는 교실에서는 스캔이 엉뚱한 곳에 맞아 위치가 수십 cm 씩 튄다. 보수적으로:
    slam['do_loop_closing'] = False                     # 2 m 안에서 도는데 '같은 곳' 판정이 오히려 튐을 만든다
    slam['correlation_search_space_dimension'] = 0.3    # 한 번에 고치는 폭 ±0.15 m (공식 ±0.25 m)
    slam['coarse_search_angle_offset'] = 0.2            # 한 번에 고치는 각 ±0.2 rad (공식 ±0.35)
    slam['link_match_minimum_response_fine'] = 0.2      # 약하게 맞은 결과는 버린다 (공식 0.1)
    return params
