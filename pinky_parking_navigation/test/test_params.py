"""로봇별 Nav2 파라미터(params.nav2_params) 검사."""
from pinky_parking_navigation.params import nav2_params


def base():
    return {
        'amcl': {'ros__parameters': {'initial_pose': [0, 0, 0]}},
        'controller_server': {'ros__parameters': {'FollowPath': {'desired_linear_vel': 0.2}}},
        'velocity_smoother': {'ros__parameters': {
            'max_velocity': [0.25, 0.0, 1.5], 'min_velocity': [-0.25, 0.0, -1.5]}},
    }


def test_initial_pose_and_original_untouched():
    original = base()
    params = nav2_params(original, 0.5, 1.0, 3.14)
    assert params['amcl']['ros__parameters']['initial_pose'] == {
        'x': 0.5, 'y': 1.0, 'z': 0.0, 'yaw': 3.14}
    assert original['amcl']['ros__parameters']['initial_pose'] == [0, 0, 0]
    assert params['velocity_smoother'] == original['velocity_smoother']   # max_speed 없으면 그대로


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
