#!/usr/bin/env python3
"""
config/docking.yaml 의 dock 하나로 인쇄용 marker (SVG, PNG) 와, 원하면 Gazebo 설비 모델을 만든다.

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
        <geometry><box><size>0.002 {plate:.4f} {plate:.4f}</size></box></geometry>
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


def quiet_zone(cfg):
    """흰 여백: 한 변의 1/4, 적어도 20 mm"""
    return max(cfg['size'] / 4.0, 0.02)


def plate_image(cfg, dictionary, px_per_m):
    """흰 판 가운데에 marker. 그림의 오른쪽 = D 의 +y, 위 = +z (로봇이 설비를 바라본 모습)"""
    side = int(round(cfg['size'] * px_per_m))
    quiet = int(round(quiet_zone(cfg) * px_per_m))
    image = np.full((side + 2 * quiet, side + 2 * quiet), 255, np.uint8)
    image[quiet:quiet + side, quiet:quiet + side] = marker_image(dictionary, cfg['marker_id'], side)
    return image


def marker_svg(cfg, dictionary, name, dictionary_name):
    """인쇄용 SVG (mm). marker, 여백, 자르는 선, 50 mm 눈금, 설명"""
    # 칸마다 한 화소인 그림 (테두리 포함). 4x4 dictionary 면 6 x 6
    bits = int(dictionary_name.split('_')[1].split('X')[0]) + 2
    cells = marker_image(dictionary, cfg['marker_id'], bits)
    size = cfg['size'] * 1000.0
    quiet = quiet_zone(cfg) * 1000.0
    cell = size / bits
    plate = size + 2 * quiet
    page_w, page_h = plate + 20.0, plate + 34.0
    x0, y0 = 10.0 + quiet, 10.0 + quiet
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{page_w:.2f}mm" height="{page_h:.2f}mm" '
           f'viewBox="0 0 {page_w:.2f} {page_h:.2f}">',
           f'<rect x="0" y="0" width="{page_w:.2f}" height="{page_h:.2f}" fill="white"/>',
           f'<rect x="10" y="10" width="{plate:.2f}" height="{plate:.2f}" fill="white" stroke="#999" '
           'stroke-width="0.15" stroke-dasharray="1.5,1.5"/>',
           f'<rect x="{x0:.3f}" y="{y0:.3f}" width="{size:.3f}" height="{size:.3f}" fill="black"/>']
    for r in range(bits):
        for c in range(bits):
            if cells[r, c] > 127:
                out.append(f'<rect x="{x0 + c * cell:.3f}" y="{y0 + r * cell:.3f}" width="{cell:.3f}" '
                           f'height="{cell:.3f}" fill="white"/>')
    ty = 10.0 + plate + 6.0
    out += [f'<text x="10" y="{ty:.1f}" font-family="sans-serif" font-size="3">{name}: {dictionary_name} id '
            f'{cfg["marker_id"]}, {size:.1f} mm (black square), center {cfg["height"] * 1000:.0f} mm above the floor'
            '</text>',
            f'<text x="10" y="{ty + 5:.1f}" font-family="sans-serif" font-size="3">print at 100 % (no fit to page). '
            'check: this bar is 50 mm</text>',
            f'<rect x="10" y="{ty + 8:.1f}" width="50" height="1.2" fill="black"/>',
            f'<rect x="10" y="{ty + 7:.1f}" width="0.3" height="3.2" fill="black"/>',
            f'<rect x="59.7" y="{ty + 7:.1f}" width="0.3" height="3.2" fill="black"/>',
            '</svg>']
    return '\n'.join(out) + '\n'


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
    cfg = config['docks'][args.dock]
    cfg.setdefault('offset', 0.0)
    cfg.setdefault('lateral', 0.0)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config['dictionary']))
    out = os.path.splitext(args.out)[0]
    image = plate_image(cfg, dictionary, args.px_per_m)
    cv2.imwrite(out + '.png', image)
    with open(out + '.svg', 'w') as f:
        f.write(marker_svg(cfg, dictionary, args.dock, config['dictionary']))
    print(f"dock '{args.dock}': marker {cfg['marker_id']} ({config['dictionary']}), {cfg['size'] * 1000:.1f} mm + "
          f"{quiet_zone(cfg) * 1000:.0f} mm quiet zone, center {cfg['height']:.3f} m high -> {out}.svg (print this "
          f'at 100 %), {out}.png ({image.shape[1]} px, {args.px_per_m * 0.0254:.0f} dpi)')

    if args.gazebo_model:
        model = os.path.basename(os.path.normpath(args.gazebo_model))
        texture = f'aruco_{args.dock}.png'
        os.makedirs(os.path.join(args.gazebo_model, 'materials', 'scripts'), exist_ok=True)
        os.makedirs(os.path.join(args.gazebo_model, 'materials', 'textures'), exist_ok=True)
        cv2.imwrite(os.path.join(args.gazebo_model, 'materials', 'textures', texture),
                    plate_image(cfg, dictionary, 8000.0))
        with open(os.path.join(args.gazebo_model, 'materials', 'scripts', 'aruco_dock.material'), 'w') as f:
            f.write(MATERIAL.format(name=args.dock, texture=texture))
        offset = cfg['offset']
        body_d, body_w, body_h = 0.35, 0.50, max(0.45, cfg['height'] + cfg['size'])
        plate = cfg['size'] + 2 * quiet_zone(cfg)
        base = BASE.format(cx=-offset / 2, d=offset, w=body_w, grey=GREY.format('DarkGrey')) if offset > 0.02 else ''
        with open(os.path.join(args.gazebo_model, 'model.sdf'), 'w') as f:
            f.write(MODEL.format(
                model=model, dock=args.dock, marker_id=cfg['marker_id'], size=cfg['size'], height=cfg['height'],
                marker_x=-offset, grey=GREY.format('Grey'), base=base,
                # marker 판(두께 2 mm)의 앞면이 정확히 x = -offset 에 오게, 설비는 그 뒤에
                body_cx=-offset - 0.002 - body_d / 2, body_cz=body_h / 2, body_d=body_d, body_w=body_w, body_h=body_h,
                plate_cx=-offset - 0.001, plate_cy=cfg['lateral'], plate_cz=cfg['height'], plate=plate))
        with open(os.path.join(args.gazebo_model, 'model.config'), 'w') as f:
            f.write(CONFIG.format(model=model, dock=args.dock, marker_id=cfg['marker_id']))
        print(f'Gazebo model {model} -> {args.gazebo_model}')


if __name__ == '__main__':
    main()
