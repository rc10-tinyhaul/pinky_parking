"""로봇별 Nav2 파라미터(params.nav2_params) 검사."""
from pinky_parking_navigation.params import nav2_params

OFFICIAL = '[[0.06, 0.06], [0.06, -0.06], [-0.06, -0.06], [-0.06, 0.06]]'


def base():
    return {
        'amcl': {'ros__parameters': {'initial_pose': [0, 0, 0]}},
        'controller_server': {'ros__parameters': {'FollowPath': {'desired_linear_vel': 0.2}}},
        'velocity_smoother': {'ros__parameters': {
            'max_velocity': [0.25, 0.0, 1.5], 'min_velocity': [-0.25, 0.0, -1.5]}},
        'local_costmap': {'local_costmap': {'ros__parameters': {'footprint': OFFICIAL}}},
        'global_costmap': {'global_costmap': {'ros__parameters': {'footprint': OFFICIAL}}},
    }


def footprints(params):
    costmaps = ('local_costmap', 'global_costmap')
    return [params[c][c]['ros__parameters']['footprint'] for c in costmaps]


def test_initial_pose_and_original_untouched():
    original = base()
    params = nav2_params(original, 0.5, 1.0, 3.14)
    assert params['amcl']['ros__parameters']['initial_pose'] == {
        'x': 0.5, 'y': 1.0, 'z': 0.0, 'yaw': 3.14}
    assert original['amcl']['ros__parameters']['initial_pose'] == [0, 0, 0]
    assert params['velocity_smoother'] == original['velocity_smoother']   # max_speed 없으면 그대로
    assert footprints(params) == [OFFICIAL, OFFICIAL]                     # footprint 없으면 공식 값


def test_trailer_footprint_on_both_costmaps():
    poly = [[0.06, 0.06], [0.06, -0.06], [-0.175, -0.06], [-0.175, 0.06]]
    params = nav2_params(base(), 0.0, 0.0, 0.0, footprint=poly)
    expected = '[[0.06, 0.06], [0.06, -0.06], [-0.175, -0.06], [-0.175, 0.06]]'
    assert footprints(params) == [expected, expected]


def test_max_speed_lowers_controller_and_smoother():
    params = nav2_params(base(), 0.0, 0.0, 0.0, max_speed=0.1)
    follow = params['controller_server']['ros__parameters']['FollowPath']
    assert follow['desired_linear_vel'] == 0.1
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [0.1, 0.0, 1.5]
    assert smoother['min_velocity'] == [-0.1, 0.0, -1.5]


def test_max_speed_never_raises():
    params = nav2_params(base(), 0.0, 0.0, 0.0, max_speed=1.0)
    follow = params['controller_server']['ros__parameters']['FollowPath']
    assert follow['desired_linear_vel'] == 0.2
    assert params['velocity_smoother']['ros__parameters']['max_velocity'][0] == 0.25


def test_goal_tolerance_and_max_turn():
    params = nav2_params(base(), 0.0, 0.0, 0.0, goal_tolerance=0.08, max_turn=0.8)
    controller = params['controller_server']['ros__parameters']
    assert controller['general_goal_checker']['xy_goal_tolerance'] == 0.08
    assert controller['FollowPath']['rotate_to_heading_angular_vel'] <= 0.8
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][2] == 0.8 and smoother['min_velocity'][2] == -0.8
    assert smoother['max_velocity'][0] == 0.25          # 직진 상한은 그대로
    untouched = nav2_params(base(), 0.0, 0.0, 0.0)
    assert untouched['velocity_smoother']['ros__parameters']['max_velocity'][2] == 1.5


def test_lookahead_fixes_distance_and_shortens_approach():
    follow = nav2_params(base(), 0.0, 0.0, 0.0, lookahead=0.2)[
        'controller_server']['ros__parameters']['FollowPath']
    assert follow['lookahead_dist'] == 0.2 and follow['min_lookahead_dist'] == 0.2
    assert follow['use_velocity_scaled_lookahead_dist'] is False
    assert follow['approach_velocity_scaling_dist'] == 0.25


def test_slam_params_own_frame_and_frequent_matching():
    from pinky_parking_navigation.params import slam_params
    original = {'slam_toolbox': {'ros__parameters': {'minimum_travel_distance': 0.5}}}
    slam = slam_params(original)['slam_toolbox']['ros__parameters']
    assert slam['map_frame'] == 'slam'                  # map -> slam 은 launch 가 시작 자리로 잇는다
    assert slam['minimum_travel_distance'] == 0.1 and slam['mode'] == 'mapping'
    assert original['slam_toolbox']['ros__parameters']['minimum_travel_distance'] == 0.5


def test_waypoint_follower_stops_on_failure():
    params = nav2_params(base(), 0.0, 0.0, 0.0)
    assert params['waypoint_follower']['ros__parameters']['stop_on_failure'] is True


def test_progress_checker_fits_short_arena_moves():
    """출차 첫 구간(0.3 m)보다 짧은 거리로 진행을 본다 (공식 0.5 m 는 채울 수 없다)."""
    checker = nav2_params(base(), 0.0, 0.0, 0.0)['controller_server']['ros__parameters'][
        'progress_checker']
    assert checker['required_movement_radius'] < 0.3
    assert checker['movement_time_allowance'] >= 15.0
