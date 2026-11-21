#!/usr/bin/env python3
"""
위치 보정용 ArUco 마커를 A4 인쇄용 그림(PNG, 300 dpi)으로 만든다.

  python3 scripts/make_markers.py --ids 1-8 --size 80 --out docs/markers
  → docs/markers/markers_p1.png, ...   (A4 한 장에 마커 2개. 100 % 크기로 인쇄: '용지에 맞춤' 끄기)

마커마다 번호와 크기, 위쪽 방향 화살표를 적는다. 쪽마다 100 mm 눈금을 그려 인쇄 크기를 확인한다.
크기(검은 정사각형 한 변)는 pinky_parking_navigation/config/markers.yaml 의 값과 같아야 한다.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

DPI = 300
A4 = (2480, 3508)            # 300 dpi 에서 가로, 세로 [px]


def mm(value):
    return int(round(value / 25.4 * DPI))


def marker_image(dictionary, marker_id, side_px):
    table = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(table, marker_id, side_px)
    return cv2.aruco.drawMarker(table, marker_id, side_px)


def parse_ids(text):
    ids = []
    for part in text.split(','):
        if '-' in part:
            low, high = part.split('-')
            ids += list(range(int(low), int(high) + 1))
        elif part:
            ids.append(int(part))
    return ids


def main():
    parser = argparse.ArgumentParser(description='ArUco 마커 인쇄용 그림 만들기')
    parser.add_argument('--ids', default='1-8', help='번호: 1-8 또는 1,2,5')
    parser.add_argument('--size', type=float, default=80.0, help='검은 정사각형 한 변 [mm]')
    parser.add_argument('--dictionary', default='DICT_4X4_50')
    parser.add_argument('--out', default='docs/markers')
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    side, margin = mm(args.size), mm(args.size * 0.25)       # 흰 여백: 마커의 1/4 (검출에 꼭 필요)
    ids = parse_ids(args.ids)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for page, start in enumerate(range(0, len(ids), 2), 1):
        sheet = np.full((A4[1], A4[0]), 255, dtype=np.uint8)
        for slot, marker_id in enumerate(ids[start:start + 2]):
            cx, cy = A4[0] // 2, A4[1] // 4 + slot * (A4[1] // 2 - mm(14)) - mm(8)
            x0, y0 = cx - side // 2, cy - side // 2
            sheet[y0:y0 + side, x0:x0 + side] = marker_image(args.dictionary, marker_id, side)
            box = (x0 - margin, y0 - margin, x0 + side + margin, y0 + side + margin)
            cv2.rectangle(sheet, box[:2], box[2:], 160, 2)                     # 자르는 선
            label = f'ID {marker_id}   {args.size:.0f} mm   {args.dictionary}'
            cv2.putText(sheet, label, (box[0], box[3] + mm(9)), font, 1.4, 0, 3)
            cv2.arrowedLine(sheet, (box[2] + mm(8), y0 + side), (box[2] + mm(8), y0), 0, 5,
                            tipLength=0.08)
            cv2.putText(sheet, 'UP', (box[2] + mm(11), y0 + side // 2), font, 1.2, 0, 3)
        rx, ry = mm(15), A4[1] - mm(12)                                         # 100 mm 눈금
        cv2.line(sheet, (rx, ry), (rx + mm(100), ry), 0, 4)
        for tick in range(0, 101, 10):
            cv2.line(sheet, (rx + mm(tick), ry - mm(3)), (rx + mm(tick), ry), 0, 3)
        cv2.putText(sheet, '100 mm  (print at 100%, no fit-to-page)', (rx + mm(104), ry),
                    font, 1.1, 0, 3)
        path = out / f'markers_p{page}.png'
        cv2.imwrite(str(path), sheet)
        print(path, ids[start:start + 2])


if __name__ == '__main__':
    main()
