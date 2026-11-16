#!/usr/bin/env python3
"""
config/docking.yaml 의 dock 하나로 인쇄용 marker (SVG, PNG) 와, 원하면 Gazebo 설비 모델을 만든다.
dock 에 marker 가 여럿이면 (markers: 목록) 한 장에 모두 찍는다: marker 사이의 간격이 인쇄로 정해져서 따로 붙일 때보다 정확하다.

  python3 make_aruco_dock.py ../config/docking.yaml dock_a marker_dock_a          # marker_dock_a.svg, .png
  python3 make_aruco_dock.py ../config/docking.yaml dock_a marker_dock_a \\
      --gazebo-model ../../../simple_line/simple_line_bringup/models/aruco_dock

인쇄 (실제 로봇):
  - SVG 를 쓴다 (mm 단위라 크기가 정확하다). 브라우저나 Inkscape 에서 "실제 크기 / 100 %" 로 인쇄한다
    ("용지에 맞춤" 을 끈다). 종이에 같이 찍히는 50 mm 눈금을 자로 재서 배율을 확인한다.
  - marker 한 변(검은 테두리 바깥)을 캘리퍼로 재서 yaml 의 size 와 다르면 yaml 을 고친다.
  - 무광 용지 (반사가 있으면 검출이 흔들린다), 레이저 프린터, 단단하고 평평한 판(아크릴, 알루미늄 복합판)에 붙인다.
    휘거나 들뜨면 그만큼 자세가 틀린다. 흰 여백(quiet zone)을 자르지 않는다: 한 변의 1/4 이상 남긴다.
  - 붙이는 높이: marker 중심 = 카메라 렌즈 높이 (yaml 의 height). 수평은 수준기로.

Gazebo 모델 (원점 = dock 좌표계 D 의 원점: 바닥, 도킹 면의 가운데, +x = 로봇이 다가오는 쪽):
  - 설비: marker 면 뒤의 상자 (폭 0.5, 깊이 0.35, 높이 0.45 m). 앞면에 marker 판
  - offset > 0 이면 (marker 가 도킹 면보다 뒤) 그 사이에 낮은 받침 (높이 0.05 m)
"""

import argparse
import os

import cv2
import numpy as np
import yaml

# filtering trilinear: 실제 렌즈처럼 칸 경계가 조금 흐리게 (서브픽셀 모서리가 더 자연스럽다)
MATERIAL = '''material ArucoDock/{name}
{{
  technique
  {{
    pass
    {{
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1
      emissive 0.5 0.5 0.5 1
      texture_unit
      {{
        texture {texture}
        filtering trilinear
      }}
    }}
  }}
}}
'''

GREY = ('<material><script><uri>file://media/materials/scripts/gazebo.material</uri>'
        '<name>Gazebo/{}</name></script></material>')

MODEL = '''<?xml version="1.0"?>
<!-- make_aruco_dock.py 가 만든 설비 모형 (dock {dock}, marker {marker_id}, 한 변 {size} m).
     원점 = 바닥에서 도킹 면의 가운데, +x = 로봇이 다가오는 쪽, z = 위. 월드에서는 +x 가 방을 보게 놓는다.
     marker 는 x = {marker_x} 의 면에, 중심 높이 {height} m -->
<sdf version="1.6">
  <model name="{model}">
    <static>true</static>
    <link name="body">
      <collision name="machine_collision">
        <pose>{body_cx:.4f} 0 {body_cz:.4f} 0 0 0</pose>
        <geometry><box><size>{body_d:.4f} {body_w:.4f} {body_h:.4f}</size></box></geometry>
      </collision>
      <visual name="machine_visual">
        <pose>{body_cx:.4f} 0 {body_cz:.4f} 0 0 0</pose>
        <geometry><box><size>{body_d:.4f} {body_w:.4f} {body_h:.4f}</size></box></geometry>
        {grey}
      </visual>{base}
      <visual name="marker_visual">
        <pose>{plate_cx:.4f} {plate_cy:.4f} {plate_cz:.4f} 0 0 0</pose>
        <geometry><box><size>0.002 {plate_w:.4f} {plate_h:.4f}</size></box></geometry>
        <material>
          <script>
            <uri>model://{model}/materials/scripts</uri>
            <uri>model://{model}/materials/textures</uri>
            <name>ArucoDock/{dock}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''

BASE = '''
      <collision name="base_collision">
        <pose>{cx:.4f} 0 0.025 0 0 0</pose>
        <geometry><box><size>{d:.4f} {w:.4f} 0.05</size></box></geometry>
      </collision>
      <visual name="base_visual">
        <pose>{cx:.4f} 0 0.025 0 0 0</pose>
        <geometry><box><size>{d:.4f} {w:.4f} 0.05</size></box></geometry>
        {grey}
      </visual>'''

CONFIG = '''<?xml version="1.0"?>
<model>
  <name>{model}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>Machine mock-up with ArUco marker {marker_id} ({dock}) on its docking face (bt_mission, DOCKING.md)</description>
</model>
'''


def marker_image(dictionary, marker_id, side):
    if hasattr(cv2.aruco, 'generateImageMarker'):          # OpenCV 4.7+
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    return cv2.aruco.drawMarker(dictionary, marker_id, side)


def dock_markers(cfg):
    """docking.yaml 의 dock 하나 -> marker 목록 [{marker_id, size, height, offset, lateral}].
    marker 하나 (marker_id: ...) 와 여럿 (markers: [{id: ...}, ...]) 을 모두 읽는다 (aruco_detector.py 의 Dock 과 같다)"""
    common = {k: cfg[k] for k in ('size', 'height', 'offset', 'lateral') if k in cfg}
    if 'markers' in cfg:
        entries = [dict(common, **{('marker_id' if k == 'id' else k): v for k, v in m.items()}) for m in cfg['markers']]
    else:
        entries = [dict(common, marker_id=cfg['marker_id'])]
    for e in entries:
        e.setdefault('offset', 0.0)
        e.setdefault('lateral', 0.0)
    return entries


def plate_bounds(markers):
    """marker 를 모두 담는 흰 판 (D 의 y, z): (y0, y1, z0, z1). 흰 여백은 한 변의 1/4, 적어도 20 mm"""
    quiet = max(max(m['size'] for m in markers) / 4.0, 0.02)
    return (min(m['lateral'] - m['size'] / 2 for m in markers) - quiet,
            max(m['lateral'] + m['size'] / 2 for m in markers) + quiet,
            min(m['height'] - m['size'] / 2 for m in markers) - quiet,
            max(m['height'] + m['size'] / 2 for m in markers) + quiet)


def plate_image(markers, dictionary, px_per_m):
    """흰 판에 marker 들. 그림의 오른쪽 = D 의 +y, 위 = +z (로봇이 설비를 바라본 모습)"""
    y0, y1, z0, z1 = plate_bounds(markers)
    image = np.full((int(round((z1 - z0) * px_per_m)), int(round((y1 - y0) * px_per_m))), 255, np.uint8)
    for m in markers:
        side = int(round(m['size'] * px_per_m))
        col = int(round((m['lateral'] - m['size'] / 2 - y0) * px_per_m))
        row = int(round((z1 - m['height'] - m['size'] / 2) * px_per_m))
        image[row:row + side, col:col + side] = marker_image(dictionary, m['marker_id'], side)
    return image


def marker_svg(markers, dictionary, name, dictionary_name):
    """인쇄용 SVG (mm). marker 들(서로의 간격이 그대로 찍힌다), 여백, 자르는 선, 50 mm 눈금, 설명"""
    # 칸마다 한 화소인 그림 (테두리 포함). 4x4 dictionary 면 6 x 6
    bits = int(dictionary_name.split('_')[1].split('X')[0]) + 2
    y0, y1, z0, z1 = (v * 1000.0 for v in plate_bounds(markers))
    plate_w, plate_h = y1 - y0, z1 - z0
    page_w, page_h = plate_w + 20.0, plate_h + 39.0
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{page_w:.2f}mm" height="{page_h:.2f}mm" '
           f'viewBox="0 0 {page_w:.2f} {page_h:.2f}">',
           f'<rect x="0" y="0" width="{page_w:.2f}" height="{page_h:.2f}" fill="white"/>',
           f'<rect x="10" y="10" width="{plate_w:.2f}" height="{plate_h:.2f}" fill="white" stroke="#999" '
           'stroke-width="0.15" stroke-dasharray="1.5,1.5"/>']
    for m in markers:
        size = m['size'] * 1000.0
        cell = size / bits
        x0 = 10.0 + (m['lateral'] * 1000.0 - size / 2 - y0)
        top = 10.0 + (z1 - m['height'] * 1000.0 - size / 2)
        cells = marker_image(dictionary, m['marker_id'], bits)
        out.append(f'<rect x="{x0:.3f}" y="{top:.3f}" width="{size:.3f}" height="{size:.3f}" fill="black"/>')
        for r in range(bits):
            for c in range(bits):
                if cells[r, c] > 127:
                    out.append(f'<rect x="{x0 + c * cell:.3f}" y="{top + r * cell:.3f}" width="{cell:.3f}" '
                               f'height="{cell:.3f}" fill="white"/>')
        out.append(f'<text x="{x0 + size / 2:.2f}" y="{top + size + 6:.2f}" font-family="sans-serif" font-size="3" '
                   f'text-anchor="middle" fill="#666">id {m["marker_id"]}</text>')
    ty = 10.0 + plate_h + 6.0
    ids = ', '.join(str(m['marker_id']) for m in markers)
    sizes = ' / '.join(sorted({f"{m['size'] * 1000:.1f}" for m in markers}))
    lines = [f'{name}: {dictionary_name} id {ids}, {sizes} mm (black square), centers '
             f'{markers[0]["height"] * 1000:.0f} mm above the floor']
    if len(markers) > 1:
        gaps = ', '.join(f'{(b["lateral"] - a["lateral"]) * 1000:.1f}' for a, b in zip(markers, markers[1:]))
        lines.append(f'center to center {gaps} mm: keep the markers on this one sheet, level')
    lines.append('print at 100 % (no fit to page). check: this bar is 50 mm')
    for i, line in enumerate(lines):
        out.append(f'<text x="10" y="{ty + 5 * i:.1f}" font-family="sans-serif" font-size="3">{line}</text>')
    by = ty + 5 * len(lines) - 2
    out += [f'<rect x="10" y="{by:.1f}" width="50" height="1.2" fill="black"/>',
            f'<rect x="10" y="{by - 1:.1f}" width="0.3" height="3.2" fill="black"/>',
            f'<rect x="59.7" y="{by - 1:.1f}" width="0.3" height="3.2" fill="black"/>',
            '</svg>']
    return '\n'.join(out) + '\n', page_w, page_h


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('config', help='docking.yaml')
    parser.add_argument('dock', help='dock name in the yaml, e.g. dock_a')
    parser.add_argument('out', help='output name without extension (writes .svg and .png)')
    parser.add_argument('--px-per-m', type=float, default=8000.0, help='PNG resolution (8000 = 203 dpi)')
    parser.add_argument('--gazebo-model', help='also write a Gazebo model to this directory')
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)
    markers = dock_markers(config['docks'][args.dock])
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config['dictionary']))
    out = os.path.splitext(args.out)[0]
    image = plate_image(markers, dictionary, args.px_per_m)
    cv2.imwrite(out + '.png', image)
    svg, page_w, page_h = marker_svg(markers, dictionary, args.dock, config['dictionary'])
    with open(out + '.svg', 'w') as f:
        f.write(svg)
    ids = ', '.join(str(m['marker_id']) for m in markers)
    paper = 'A4 portrait' if page_w <= 200 and page_h <= 287 else 'A4 landscape' if page_w <= 287 and page_h <= 200 \
        else 'larger than A4: print on A3 or put the markers closer'
    sizes = ' / '.join('%.1f' % (m['size'] * 1000) for m in markers)
    print(f"dock '{args.dock}': marker {ids} ({config['dictionary']}), {sizes} mm, center "
          f"{markers[0]['height']:.3f} m high -> "
          f'{out}.svg ({page_w:.0f} x {page_h:.0f} mm, {paper}. print this at 100 %), {out}.png '
          f'({image.shape[1]} x {image.shape[0]} px, {args.px_per_m * 0.0254:.0f} dpi)')

    if args.gazebo_model:
        model = os.path.basename(os.path.normpath(args.gazebo_model))
        texture = f'aruco_{args.dock}.png'
        os.makedirs(os.path.join(args.gazebo_model, 'materials', 'scripts'), exist_ok=True)
        os.makedirs(os.path.join(args.gazebo_model, 'materials', 'textures'), exist_ok=True)
        cv2.imwrite(os.path.join(args.gazebo_model, 'materials', 'textures', texture),
                    plate_image(markers, dictionary, 8000.0))
        with open(os.path.join(args.gazebo_model, 'materials', 'scripts', 'aruco_dock.material'), 'w') as f:
            f.write(MATERIAL.format(name=args.dock, texture=texture))
        offset = markers[0]['offset']
        y0, y1, z0, z1 = plate_bounds(markers)
        body_d, body_w, body_h = 0.35, max(0.50, y1 - y0 + 0.1), max(0.45, z1 + 0.05)
        base = BASE.format(cx=-offset / 2, d=offset, w=body_w, grey=GREY.format('DarkGrey')) if offset > 0.02 else ''
        with open(os.path.join(args.gazebo_model, 'model.sdf'), 'w') as f:
            f.write(MODEL.format(
                model=model, dock=args.dock, marker_id=ids, size=' / '.join(str(m['size']) for m in markers),
                height=markers[0]['height'], marker_x=-offset, grey=GREY.format('Grey'), base=base,
                # marker 판(두께 2 mm)의 앞면이 정확히 x = -offset 에 오게, 설비는 그 뒤에
                body_cx=-offset - 0.002 - body_d / 2, body_cz=body_h / 2, body_d=body_d, body_w=body_w, body_h=body_h,
                plate_cx=-offset - 0.001, plate_cy=(y0 + y1) / 2, plate_cz=(z0 + z1) / 2,
                plate_w=y1 - y0, plate_h=z1 - z0))
        with open(os.path.join(args.gazebo_model, 'model.config'), 'w') as f:
            f.write(CONFIG.format(model=model, dock=args.dock, marker_id=ids))
        print(f'Gazebo model {model} -> {args.gazebo_model}')


if __name__ == '__main__':
    main()
