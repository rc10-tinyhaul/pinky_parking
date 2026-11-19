"""
Nav2 지도(map_server 의 YAML + PGM)를 대시보드가 그릴 수 있는 형태로 읽는다.

cells 는 한 칸에 한 글자: 'o' 장애물, 'f' 빈 곳, 'u' 모름. 맨 윗줄(y 가 가장 큰 줄)부터 차례로.
"""
from pathlib import Path

import yaml


def read_pgm(data):
    """P5 PGM 바이트 → (width, height, maxval, pixels)."""
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    if tokens[0] != b'P5':
        raise ValueError('P5 PGM 만 읽습니다')
    width, height, maxval = (int(t) for t in tokens[1:])
    pixels = data[pos + 1:pos + 1 + width * height]
    if len(pixels) != width * height:
        raise ValueError('PGM 크기가 맞지 않습니다')
    return width, height, maxval, pixels


def classify(pixels, maxval, negate, occupied_thresh, free_thresh):
    """map_server 의 trinary 규칙으로 칸을 나눈다."""
    out = []
    for value in pixels:
        p = value / maxval if negate else (maxval - value) / maxval
        out.append('o' if p > occupied_thresh else 'f' if p < free_thresh else 'u')
    return ''.join(out)


def load_map(yaml_path):
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    width, height, maxval, pixels = read_pgm((yaml_path.parent / meta['image']).read_bytes())
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    return {
        'width': width, 'height': height, 'resolution': float(meta['resolution']),
        'origin': {'x': float(origin[0]), 'y': float(origin[1])},
        'cells': classify(pixels, maxval, int(meta.get('negate', 0)),
                          float(meta.get('occupied_thresh', 0.65)),
                          float(meta.get('free_thresh', 0.196))),
    }
