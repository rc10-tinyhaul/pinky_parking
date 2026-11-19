"""
로봇 목록(config/robots.yaml) 읽기.

robots.yaml 형식:

    world_name: pinky_factory.world
    robots:
      - {name: pinky1, domain_id: 11, x: 0.0, y: 0.0, yaw: 0.0, vehicle: basic}

name 은 Gazebo 토픽 접두어(pinky1/scan), domain_id 는 그 로봇의 ROS_DOMAIN_ID,
x·y [m]·yaw [rad] 는 스폰 위치이자 AMCL 초기 위치다. vehicle 은 차종
(pinky_parking_description/config/vehicles.yaml), 없으면 basic.
"""
from pathlib import Path

import yaml


def load_robots(path):
    """(world_name, robots) 를 돌려준다."""
    data = yaml.safe_load(Path(path).read_text())
    robots = data.get('robots') or []
    for key in ('name', 'domain_id'):
        values = [r.get(key) for r in robots]
        if None in values:
            raise ValueError(f'robots.yaml: 모든 로봇에 {key} 가 있어야 합니다')
        if len(set(values)) != len(values):
            raise ValueError(f'robots.yaml: {key} 가 겹칩니다: {values}')
    for r in robots:
        r.setdefault('x', 0.0)
        r.setdefault('y', 0.0)
        r.setdefault('yaw', 0.0)
        r.setdefault('vehicle', 'basic')
    return data.get('world_name', 'pinky_factory.world'), robots
