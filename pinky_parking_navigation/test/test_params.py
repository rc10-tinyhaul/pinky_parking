"""로봇별 Nav2 파라미터(params.nav2_params) 검사."""
from pinky_parking_navigation.params import nav2_params

OFFICIAL = '[[0.06, 0.06], [0.06, -0.06], [-0.06, -0.06], [-0.06, 0.06]]'


def base():
    return {
        'amcl': {'ros__parameters': {'initial_pose': [0, 0, 0]}},
        'local_costmap': {'local_costmap': {'ros__parameters': {'footprint': OFFICIAL}}},
        'global_costmap': {'global_costmap': {'ros__parameters': {'footprint': OFFICIAL}}},
    }


def footprints(params):
    costmaps = ('local_costmap', 'global_costmap')
    return [params[c][c]['ros__parameters']['footprint'] for c in costmaps]


def test_initial_pose_and_official_footprint_kept():
    original = base()
    params = nav2_params(original, 0.5, 1.0, 3.14)
    assert params['amcl']['ros__parameters']['initial_pose'] == {
        'x': 0.5, 'y': 1.0, 'z': 0.0, 'yaw': 3.14}
    assert footprints(params) == [OFFICIAL, OFFICIAL]
    assert original['amcl']['ros__parameters']['initial_pose'] == [0, 0, 0]   # 원본은 그대로


def test_trailer_footprint_on_both_costmaps():
    poly = [[0.06, 0.06], [0.06, -0.06], [-0.175, -0.06], [-0.175, 0.06]]
    params = nav2_params(base(), 0.0, 0.0, 0.0, footprint=poly)
    expected = '[[0.06, 0.06], [0.06, -0.06], [-0.175, -0.06], [-0.175, 0.06]]'
    assert footprints(params) == [expected, expected]
