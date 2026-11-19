"""
차종(config/vehicles.yaml)과 그 차종의 URDF.

  from pinky_parking_description.vehicles import vehicle_urdf
  urdf = vehicle_urdf('trailer10', namespace='pinky1/', is_sim=True)
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

LIDAR_HEIGHT = 0.125   # 바닥에서 LiDAR 스캔 면까지 [m] (공식 모델)
KEYS = ('length', 'width', 'top', 'clearance', 'mass')


def share():
    return Path(get_package_share_directory('pinky_parking_description'))


def load_vehicles(path=None):
    """{차종 이름: {length, width, top, clearance, mass}} (트레일러 없는 차종은 빈 dict)."""
    path = path or share() / 'config/vehicles.yaml'
    vehicles = yaml.safe_load(Path(path).read_text())['vehicles']
    for name, spec in vehicles.items():
        spec = spec or {}
        unknown = set(spec) - set(KEYS)
        if unknown:
            raise ValueError(f'vehicles.yaml {name}: 모르는 항목 {sorted(unknown)}')
        if spec and set(spec) != set(KEYS):
            raise ValueError(f'vehicles.yaml {name}: 트레일러는 {KEYS} 가 모두 있어야 합니다')
        if spec and spec['top'] >= LIDAR_HEIGHT:
            raise ValueError(f'vehicles.yaml {name}: top 이 LiDAR 높이({LIDAR_HEIGHT} m) 이상이면 '
                             'LiDAR 가 자기 트레일러를 장애물로 봅니다')
        vehicles[name] = spec
    return vehicles


def vehicle_urdf(name, namespace='', is_sim=False, cam_tilt_deg=8, vehicles=None):
    """차종 이름의 URDF 문자열."""
    vehicles = vehicles if vehicles is not None else load_vehicles()
    if name not in vehicles:
        raise ValueError(f'없는 차종입니다: {name} (있는 차종: {", ".join(vehicles)})')
    mappings = {'namespace': namespace, 'is_sim': str(is_sim), 'cam_tilt_deg': str(cam_tilt_deg)}
    mappings.update({f'trailer_{key}': str(value) for key, value in vehicles[name].items()})
    return xacro.process_file(str(share() / 'urdf/pinky_vehicle.urdf.xacro'),
                              mappings=mappings).toxml()
