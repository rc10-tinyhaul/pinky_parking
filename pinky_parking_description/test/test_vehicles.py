"""차종 URDF 검사. xacro 와 pinky_description 이 있어야 한다 (colcon test 환경)."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from pinky_parking_description.vehicles import footprint
from pinky_parking_description.vehicles import LIDAR_HEIGHT
from pinky_parking_description.vehicles import load_vehicles
from pinky_parking_description.vehicles import vehicle_urdf
import pytest
import xacro

CONFIG = Path(__file__).resolve().parents[1] / 'config/vehicles.yaml'
BODY_REAR = -0.075     # 공식 모델 차체 뒤끝 (base_footprint 기준) [m]
BASE_LINK_Z = 0.028


def structure(urdf):
    """주석을 뺀 XML 구조 (비교용)."""
    return ET.tostring(ET.fromstring(urdf))


def trailer_box(urdf):
    """(뒤끝 x, 앞끝 x, 아랫면 z, 윗면 z, 폭) base_footprint 기준."""
    root = ET.fromstring(urdf)
    joint = next(j for j in root.iter('joint') if j.find('child').get('link') == 'trailer_link')
    x, _, z = map(float, joint.find('origin').get('xyz').split())
    length, width, height = map(float, root.find(
        "link[@name='trailer_link']/collision/geometry/box").get('size').split())
    z += BASE_LINK_Z
    return x - length / 2, x + length / 2, z - height / 2, z + height / 2, width


def test_basic_is_the_official_robot():
    official = xacro.process_file(
        str(Path(get_package_share_directory('pinky_description')) / 'urdf/robot.urdf.xacro'),
        mappings={'namespace': 'pinky1/', 'is_sim': 'True', 'cam_tilt_deg': '8'}).toxml()
    basic = vehicle_urdf('T00', namespace='pinky1/', is_sim=True,
                         vehicles=load_vehicles(CONFIG))
    assert structure(basic) == structure(official)


@pytest.mark.parametrize('name', ['T10', 'T15'])
def test_trailer_behind_body_and_below_lidar(name):
    spec = load_vehicles(CONFIG)[name]
    rear, front, bottom, top, width = trailer_box(
        vehicle_urdf(name, namespace='pinky1/', is_sim=True, vehicles=load_vehicles(CONFIG)))
    assert front == pytest.approx(BODY_REAR, abs=1e-6)               # 차체 뒤끝에 붙음
    assert rear == pytest.approx(BODY_REAR - spec['length'], abs=1e-6)
    assert bottom == pytest.approx(spec['clearance'], abs=1e-6)      # 바닥에서 떠 있음
    assert top == pytest.approx(spec['top'], abs=1e-6)
    assert top < LIDAR_HEIGHT                                        # LiDAR 가 보지 않음
    assert width == pytest.approx(spec['width'])


def test_unknown_vehicle_and_too_tall_trailer(tmp_path):
    with pytest.raises(ValueError, match='없는 차종'):
        vehicle_urdf('truck', vehicles=load_vehicles(CONFIG))
    bad = tmp_path / 'vehicles.yaml'
    bad.write_text('vehicles:\n  tall: {length: 0.1, width: 0.1, top: 0.2, clearance: 0.01, '
                   'mass: 0.05}\n')
    with pytest.raises(ValueError, match='LiDAR'):
        load_vehicles(bad)


def test_footprint():
    vehicles = load_vehicles(CONFIG)
    assert footprint(vehicles['T00']) is None            # 공식 Nav2 값을 그대로 씀
    poly = footprint(vehicles['T10'])
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    assert min(xs) == pytest.approx(BODY_REAR - 0.10)       # 트레일러 끝까지
    assert max(xs) == pytest.approx(0.06)                   # 앞은 공식 값
    assert max(ys) == pytest.approx(0.06) and min(ys) == pytest.approx(-0.06)
