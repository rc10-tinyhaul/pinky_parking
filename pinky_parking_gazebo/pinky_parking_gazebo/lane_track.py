"""
차선 시험 트랙: 치수(config/lane_track.yaml)에서 Gazebo 월드를 만들고, 중심선까지 거리를 잰다.

트랙은 직선 두 개와 반원 두 개로 된 닫힌 곡선이다. 원점이 중심이고 직선은 y 방향이다.
반경 r 인 곡선은 "선분 (0, -L/2)~(0, L/2) 에서 거리가 r 인 점들"이므로, 한 점이 곡선에서
벗어난 거리는 (선분까지 거리 - r) 이다.

  ros2 run pinky_parking_gazebo make_lane_world > worlds/lane_test.sdf
"""
import argparse
import math
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
import yaml

SEGMENT_LENGTH = 0.03   # 곡선 테이프를 이 길이의 직선 조각으로 나눈다 [m]
TAPE_HEIGHT = 0.001     # 테이프 두께. 충돌은 없고 보이기만 한다 [m]


def load_track(path=None):
    if path is None:
        path = Path(get_package_share_directory('pinky_parking_gazebo')) / 'config/lane_track.yaml'
    return yaml.safe_load(Path(path).read_text())


def tape_radii(track):
    """테이프 선의 곡선 반경 목록 (차선 중심선의 양옆, 겹치는 것은 하나로)."""
    half = track['lane_width'] / 2
    radii = {round(r + side * half, 6) for r in track['lane_radii'] for side in (-1, 1)}
    return sorted(radii)


def core_distance(track, x, y):
    """트랙 가운데 선분까지 거리."""
    half = track['straight_length'] / 2
    return math.hypot(x, y - max(-half, min(half, y)))


def lane_offset(track, x, y, lane=None):
    """
    (차선 번호, 중심선에서 벗어난 거리) 를 돌려준다.

    차선 번호는 1 부터. lane 을 안 주면 가장 가까운 차선을 고른다. 거리는 바깥쪽이 +.
    """
    distance = core_distance(track, x, y)
    radii = track['lane_radii']
    if lane is None:
        lane = min(range(len(radii)), key=lambda i: abs(distance - radii[i])) + 1
    return lane, distance - radii[lane - 1]


def tape_boxes(track):
    """테이프 조각 목록: (x, y, yaw, 길이)."""
    half = track['straight_length'] / 2
    boxes = []
    for r in tape_radii(track):
        for side in (-1, 1):
            if half > 0:
                boxes.append((side * r, 0.0, math.pi / 2, track['straight_length']))
            count = max(8, math.ceil(math.pi * r / SEGMENT_LENGTH))
            step = math.pi / count
            # 조각이 바깥쪽에서 벌어지지 않게 테이프 바깥 가장자리 기준 길이로 한다
            length = 2 * (r + track['tape_width'] / 2) * math.tan(step / 2)
            for i in range(count):
                angle = (i + 0.5) * step          # 0 ~ pi: 위쪽 반원
                x, y = r * math.cos(angle), r * math.sin(angle)
                boxes.append((side * x, side * (y + half), angle + math.pi / 2, length))
    return boxes


def world_sdf(track):
    def rgba(color):
        return ' '.join(f'{c:g}' for c in color) + ' 1'

    def material(color):
        return (f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
                '<specular>0 0 0 1</specular></material>')

    tape = material(rgba(track['tape_color']))
    floor = material(rgba(track['floor_color']))
    size = f"{{length:.4f}} {track['tape_width']:g} {TAPE_HEIGHT:g}"
    visuals = '\n'.join(f"""\
        <visual name="tape_{i}">
          <pose>{x:.4f} {y:.4f} {TAPE_HEIGHT / 2:g} 0 0 {yaw:.4f}</pose>
          <geometry><box><size>{size.format(length=length)}</size></box></geometry>
          {tape}
        </visual>""" for i, (x, y, yaw, length) in enumerate(tape_boxes(track)))
    return f"""\
<?xml version="1.0"?>
<!-- make_lane_world 가 config/lane_track.yaml 에서 만든 파일. 직접 고치지 않는다. -->
<sdf version="1.9">
  <world name="default">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          {floor}
        </visual>
      </link>
    </model>

    <model name="lane_tape">
      <static>true</static>
      <link name="link">
{visuals}
      </link>
    </model>
  </world>
</sdf>
"""


def main():
    parser = argparse.ArgumentParser(description='차선 시험 트랙 월드(SDF)를 출력')
    parser.add_argument('--track', help='트랙 치수 파일 (기본: config/lane_track.yaml)')
    args = parser.parse_args()
    sys.stdout.write(world_sdf(load_track(args.track)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
