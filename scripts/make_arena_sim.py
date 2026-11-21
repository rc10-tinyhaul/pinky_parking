#!/usr/bin/env python3
"""
경기장 시뮬레이션 만들기: 경기장 도면 좌표(pinky_parking_control/config/arena.yaml)로
Gazebo 월드와 Nav2 지도를 함께 만든다. 이 저장소의 경기장 월드와 지도는 모두 이 스크립트가 만든다.

  python3 scripts/make_arena_sim.py          # 아래에 쓴다 (저장소 루트 기준). OpenCV 가 필요하다
    pinky_parking_gazebo/worlds/arena.world
    pinky_parking_gazebo/models/aruco_<번호>/                        (마커 모델, 월드가 include)
    pinky_parking_navigation/maps/arena.yaml, arena.pgm
    pinky_parking_navigation/maps/arena_open.yaml, arena_open.pgm   (실물용, 벽 없음)

- 바닥 2000 x 2000 mm 위의 테이프, 주차 칸, 주유구역, 대기선은 그림(visual)만 있다. 라이다에는 안 보인다.
- 라이다로 위치를 잡도록(AMCL) 바닥 바깥 ROOM_MARGIN 만큼 떨어진 곳에 낮은 벽을 두르고,
  방이 돌려도 같아 보이지 않게 직사각형으로 하고 벽마다 크기가 다른 블록을 둔다.
  실물 경기장에는 없는 것이다.
- ArUco 마커(arena.yaml 의 markers)는 바닥에 붙인 그림이다. 마커마다 모델(그림 PNG 포함)을 만들어
  월드가 model://aruco_<번호> 로 부른다. 모델 경로는 world.launch.py 가 GZ_SIM_RESOURCE_PATH 에 넣는다.
- 지도는 월드 원점에서 만든 것과 같다 (지도 좌표 = 월드 좌표 = 도면 좌표).
"""
import argparse
import math
from pathlib import Path

import cv2
from make_markers import marker_image
import yaml

ROOT = Path(__file__).resolve().parents[1]
ARENA_YAML = ROOT / 'pinky_parking_control/config/arena.yaml'
WORLD_OUT = ROOT / 'pinky_parking_gazebo/worlds/arena.world'
MODELS_OUT = ROOT / 'pinky_parking_gazebo/models'
MAP_OUT = ROOT / 'pinky_parking_navigation/maps/arena'

# 바닥 가장자리에서 벽 안쪽 면까지 [m] (왼쪽, 오른쪽, 아래, 위). 정사각형이면 90° 돌린 모습과
# 구별이 안 돼 AMCL 이 방향을 잃으므로 위쪽만 더 띄워 직사각형으로 만든다.
ROOM_MARGIN = (0.5, 0.5, 0.5, 1.0)
WALL_THICK = 0.05
WALL_HEIGHT = 0.25     # 라이다 높이 약 0.1 m 보다 높게
BLOCKS = [             # (x, y, 가로, 세로) 벽에 붙인 크기가 서로 다른 블록. 벽 안쪽, 경기장 바닥 밖.
    (-1.45, 1.75, 0.30, 0.30),
    (1.10, -1.30, 0.20, 0.40),
    (0.90, 1.90, 0.60, 0.15),
    (-1.62, -0.80, 0.15, 0.50),
]
MAP_RESOLUTION = 0.02
ARC_SEGMENTS = 24      # 반원 하나를 몇 조각 직선으로 그릴지
Z = 0.0005             # 바닥 위 그림 높이 (겹침 깜빡임 방지)
MARKER_PX = 60         # 마커 그림의 칸 하나 [px] (4x4 마커 = 검은 테두리 포함 6 칸)
MARKER_QUIET = 1       # 둘레 흰 여백 [칸]. 여백이 없으면 바닥과 구별이 안 돼 검출되지 않는다
MARKER_Z = 0.004       # 마커 높이 [m]. 테이프 윗면(2 mm)과 같으면 멀리서 바닥에 가려 보이지 않는다

COLORS = {
    'board': (0.86, 0.86, 0.86), 'asphalt': (0.62, 0.62, 0.62), 'tape': (0.03, 0.03, 0.03),
    'park': (0.18, 0.62, 0.25), 'fuel': (0.85, 0.20, 0.17), 'wait': (0.11, 0.25, 0.95),
    'wall': (0.55, 0.50, 0.45), 'floor': (0.35, 0.35, 0.37),
}


def room(board):
    """벽 안쪽 사각형 (x0, y0, x1, y1)."""
    x0, y0 = board['x'], board['y']
    left, right, bottom, top = ROOM_MARGIN
    return (x0 - left, y0 - bottom, x0 + board['width'] + right, y0 + board['height'] + top)


def stadium_points(radius, straight):
    """트랙 테이프 한 줄을 이루는 꼭짓점 (반시계, 닫힌 도형)."""
    h = straight / 2.0
    pts = []
    for i in range(ARC_SEGMENTS + 1):                       # 위 반원: 오른쪽 → 왼쪽
        a = math.pi * i / ARC_SEGMENTS
        pts.append((radius * math.cos(a), h + radius * math.sin(a)))
    for i in range(ARC_SEGMENTS + 1):                       # 아래 반원: 왼쪽 → 오른쪽
        a = math.pi + math.pi * i / ARC_SEGMENTS
        pts.append((radius * math.cos(a), -h + radius * math.sin(a)))
    pts.append(pts[0])
    return pts


def box_visual(name, x, y, yaw, length, width, color, z=Z, height=0.001):
    r, g, b = color
    return f"""        <visual name="{name}">
          <pose>{x:.4f} {y:.4f} {z + height / 2:.4f} 0 0 {yaw:.4f}</pose>
          <geometry><box><size>{length:.4f} {width:.4f} {height:.4f}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>"""


def segment(name, p, q, width, color, z):
    (x0, y0), (x1, y1) = p, q
    length = math.hypot(x1 - x0, y1 - y0) + width * 0.5     # 이음매가 비지 않게 조금 겹친다
    return box_visual(name, (x0 + x1) / 2, (y0 + y1) / 2, math.atan2(y1 - y0, x1 - x0),
                      length, width, color, z)


def outline(name, cx, cy, along_y, across, color, tape, z):
    """가운데 (cx, cy), 세로(y) 길이 along_y, 가로(x) 폭 across 인 사각 테두리."""
    hx, hy = across / 2, along_y / 2
    corners = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
    return [segment(f'{name}_{i}', corners[i], corners[(i + 1) % 4], tape, color, z)
            for i in range(4)]


def filled_stadium(name, radius, straight, color, z):
    """트랙 바닥(채움): 가운데 직사각형 + 위아래 원판."""
    r, g, b = color
    h = straight / 2.0
    out = [box_visual(f'{name}_mid', 0, 0, 0, 2 * radius, straight, color, z)]
    for k, cy in (('top', h), ('bottom', -h)):
        out.append(f"""        <visual name="{name}_{k}">
          <pose>0 {cy:.4f} {z + 0.0005:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{radius:.4f}</radius><length>0.001</length></cylinder></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>""")
    return out


def lane_radii(arena):
    half = arena['lane_width'] / 2
    radii = sorted(t['radius'] for t in arena['tracks'])
    lines = [radii[0] - half] + [r + half for r in radii]
    return radii[0] - half, radii[-1] + half, lines


def make_world(arena):
    board, tape, straight = arena['board'], arena['tape_width'], arena['straight']
    inner, outer, lines = lane_radii(arena)
    bx = board['x'] + board['width'] / 2
    by = board['y'] + board['height'] / 2
    visuals = [box_visual('board', bx, by, 0, board['width'], board['height'], COLORS['board'], 0)]
    visuals += filled_stadium('asphalt', outer, straight, COLORS['asphalt'], Z)
    visuals += filled_stadium('infield', inner, straight, COLORS['board'], 2 * Z)
    for i, radius in enumerate(lines):
        pts = stadium_points(radius, straight)
        visuals += [segment(f'tape{i}_{k}', pts[k], pts[k + 1], tape, COLORS['tape'], 3 * Z)
                    for k in range(len(pts) - 1)]
    for name, s in arena['slots'].items():
        visuals += outline(name, s['x'], s['y'], s['length'], s['width'], COLORS['park'],
                           tape, 3 * Z)
    zone = arena['fuel']['zone']
    visuals += outline('fuel', zone['x'], zone['y'], zone['length'], zone['width'],
                       COLORS['fuel'], tape, 3 * Z)
    wait = arena['fuel']['wait_line']
    visuals.append(box_visual('wait_line', wait['x'], wait['y'], 0, wait['width'], 0.02,
                              COLORS['wait'], 3 * Z))

    x0, y0, x1, y1 = room(board)
    t = WALL_THICK
    walls = [  # (이름, 가운데 x, y, 가로, 세로)
        ('wall_left', x0 - t / 2, (y0 + y1) / 2, t, y1 - y0 + 2 * t),
        ('wall_right', x1 + t / 2, (y0 + y1) / 2, t, y1 - y0 + 2 * t),
        ('wall_bottom', (x0 + x1) / 2, y0 - t / 2, x1 - x0, t),
        ('wall_top', (x0 + x1) / 2, y1 + t / 2, x1 - x0, t),
    ] + [(f'block{i}', x, y, w, h) for i, (x, y, w, h) in enumerate(BLOCKS)]
    r, g, b = COLORS['wall']
    solids = []
    for name, x, y, w, h in walls:
        geometry = (f'<geometry><box><size>{w:.4f} {h:.4f} {WALL_HEIGHT}</size></box>'
                    '</geometry>')
        solids.append(f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {WALL_HEIGHT / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">{geometry}</collision>
        <visual name="visual">{geometry}
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>""")
    fr, fg, fb = COLORS['floor']
    nl = '\n'
    return f"""<?xml version="1.0"?>
<!-- scripts/make_arena_sim.py 가 pinky_parking_control/config/arena.yaml 에서 만든 경기장. -->
<!-- 직접 고치지 않는다. -->
<!-- 테이프·칸·마커는 그림만 있다. 벽과 블록은 라이다 위치 추정용으로, 실물 경기장에는 없다. -->
<sdf version="1.8">
  <world name="arena">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 3 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.2 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material>
            <ambient>{fr} {fg} {fb} 1</ambient><diffuse>{fr} {fg} {fb} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="arena_floor">
      <static>true</static>
      <link name="link">
{nl.join(visuals)}
      </link>
    </model>
{nl.join(solids)}
{nl.join(marker_includes(arena.get('markers') or {}))}
  </world>
</sdf>
"""


def marker_cells(dictionary):
    """마커 한 변의 칸 수 (검은 테두리 포함). DICT_4X4_50 이면 6."""
    return int(dictionary.split('_')[1].split('X')[0]) + 2


def marker_png(dictionary, marker_id):
    """흰 여백을 두른 마커 그림 (PNG 바이트)."""
    cells = marker_cells(dictionary)
    image = marker_image(dictionary, marker_id, cells * MARKER_PX)
    pad = MARKER_QUIET * MARKER_PX
    image = cv2.copyMakeBorder(image, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    ok, data = cv2.imencode('.png', image)
    if not ok:
        raise RuntimeError(f'마커 {marker_id} 그림을 만들지 못했습니다')
    return data.tobytes()


def marker_plate(markers):
    """흰 여백까지 포함한 마커 판 한 변 [m]."""
    cells = marker_cells(markers['dictionary'])
    return markers['size'] * (cells + 2 * MARKER_QUIET) / cells


def marker_model(markers, marker_id):
    """마커 모델 (model.config, model.sdf 텍스트). 그림은 판 윗면에만 보인다 (충돌 없음)."""
    name = f'aruco_{marker_id}'
    side = marker_plate(markers)
    config = f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>ArUco {markers['dictionary']} {marker_id}, 검은 정사각형 {markers['size'] * 1000:g} mm
    (scripts/make_arena_sim.py 가 만듦)</description>
</model>
"""
    sdf = f"""<?xml version="1.0"?>
<!-- scripts/make_arena_sim.py 가 만든 바닥 마커. 직접 고치지 않는다. -->
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>{side:.4f} {side:.4f}</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal>
            <albedo_map>model://{name}/materials/textures/{name}.png</albedo_map>
            <metalness>0</metalness><roughness>1</roughness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
    return config, sdf


def write_marker_models(markers, out_dir):
    """마커마다 모델 폴더를 만든다. 만든 번호 목록을 돌려준다."""
    out_dir = Path(out_dir)
    ids = sorted(markers['items'])
    for marker_id in ids:
        folder = out_dir / f'aruco_{marker_id}'
        textures = folder / 'materials/textures'
        textures.mkdir(parents=True, exist_ok=True)
        config, sdf = marker_model(markers, marker_id)
        (folder / 'model.config').write_text(config, encoding='utf-8')
        (folder / 'model.sdf').write_text(sdf, encoding='utf-8')
        (textures / f'aruco_{marker_id}.png').write_bytes(marker_png(markers['dictionary'], marker_id))
    return ids


def marker_includes(markers):
    """월드에 넣을 마커 include (테이프보다 위, MARKER_Z)."""
    out = []
    for marker_id, m in sorted(markers.get('items', {}).items()):
        out.append(f"""    <include>
      <uri>model://aruco_{marker_id}</uri>
      <name>aruco_{marker_id}</name>
      <pose>{m['x']:.4f} {m['y']:.4f} {MARKER_Z} 0 0 {m.get('yaw', 0.0):.4f}</pose>
    </include>""")
    return out


def make_map(arena, walls=True):
    """
    (yaml 텍스트, pgm 바이트). 벽·블록 = 검정, 벽 안 = 흰색, 벽 밖 = 모름.

    walls=False 는 실물용(arena_open): 같은 크기의 빈 바닥만. 실물에는 가상 벽이 없고,
    위치는 odom_localizer 가 바퀴 기록으로 잡으며 실제 장애물은 라이다 costmap 이 본다.
    """
    x0, y0, x1, y1 = room(arena['board'])
    t, res = WALL_THICK, MAP_RESOLUTION
    pad = t + 2 * res
    ox, oy = x0 - pad, y0 - pad
    width = int(round((x1 - x0 + 2 * pad) / res))
    height = int(round((y1 - y0 + 2 * pad) / res))
    obstacles = []
    if walls:
        obstacles = [(x0 - t, y0 - t, x0, y1 + t), (x1, y0 - t, x1 + t, y1 + t),
                     (x0 - t, y0 - t, x1 + t, y0), (x0 - t, y1, x1 + t, y1 + t)]
        obstacles += [(x - w / 2, y - h / 2, x + w / 2, y + h / 2) for x, y, w, h in BLOCKS]
    pixels = bytearray()
    for row in range(height):                     # PGM 첫 줄 = 지도 맨 위 (y 가 가장 큼)
        cy = oy + (height - row - 0.5) * res
        for col in range(width):
            cx = ox + (col + 0.5) * res
            if any(a <= cx <= c and b <= cy <= d for a, b, c, d in obstacles):
                pixels.append(0)
            elif x0 <= cx <= x1 and y0 <= cy <= y1:
                pixels.append(254)
            else:
                pixels.append(205)
    pgm = f'P5\n{width} {height}\n255\n'.encode() + bytes(pixels)
    name = 'arena' if walls else 'arena_open'
    what = '경기장 지도 (월드 arena.world 와 같은 좌표)' if walls else \
        '실물용 벽 없는 경기장 바닥 (도면 좌표, odom 위치 추정과 함께)'
    meta = (f'# scripts/make_arena_sim.py 가 만든 {what}.\n'
            f'image: {name}.pgm\nmode: trinary\nresolution: {res}\n'
            f'origin: [{ox:.3f}, {oy:.3f}, 0]\nnegate: 0\noccupied_thresh: 0.65\n'
            f'free_thresh: 0.196\n')
    return meta, pgm


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--arena', default=str(ARENA_YAML))
    parser.add_argument('--world', default=str(WORLD_OUT))
    parser.add_argument('--map', default=str(MAP_OUT), help='확장자 없이 (yaml, pgm 을 쓴다)')
    parser.add_argument('--models', default=str(MODELS_OUT), help='마커 모델 폴더')
    args = parser.parse_args()
    arena = yaml.safe_load(Path(args.arena).read_text(encoding='utf-8'))
    for path in (Path(args.world), Path(args.map)):
        path.parent.mkdir(parents=True, exist_ok=True)
    Path(args.world).write_text(make_world(arena), encoding='utf-8')
    if arena.get('markers'):
        ids = write_marker_models(arena['markers'], args.models)
        print(f'마커 모델: {args.models}/aruco_{{{",".join(map(str, ids))}}}')
    for walls, path in ((True, args.map), (False, args.map + '_open')):
        meta, pgm = make_map(arena, walls)
        Path(path + '.yaml').write_text(meta, encoding='utf-8')
        Path(path + '.pgm').write_bytes(pgm)
        print(f'지도: {path}.yaml, {path}.pgm')
    print(f'월드: {args.world}')


if __name__ == '__main__':
    main()
