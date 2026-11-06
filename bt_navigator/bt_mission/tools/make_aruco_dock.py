#!/usr/bin/env python3
"""
config/aruco_boards.yaml 의 board 하나로 인쇄용 PNG 와, 원하면 Gazebo dock 모델을 만든다.

  python3 make_aruco_dock.py ../config/aruco_boards.yaml dock_a dock_a.png
  python3 make_aruco_dock.py ../config/aruco_boards.yaml dock_a dock_a.png \\
      --gazebo-model ../../simple_line/simple_line_bringup/models/aruco_dock

인쇄: PNG 는 --px-per-m (기본 4000 = 101.6 dpi) 로 그린다. 인쇄한 뒤 marker 한 변을 자로 재서 yaml 의
size 와 다르면 고친다 (또는 aruco_detector.py 의 board_scale). 판은 휘지 않게 단단한 판(폼보드, 아크릴)에 붙인다.

Gazebo 모델 (원점 = dock 좌표계 D 의 원점: 바닥, 충전 접점 면의 가운데, +x = 방 쪽):
  - 충전대: 높이 0.05 m 의 낮은 상자 (x: -board_offset ~ 0). 앞면에 접점 판(주황)
  - 뒤판: x = -board_offset 에 세운 판. 앞면에 marker 판 그림
  레이저(높이 약 0.16 m)는 낮은 충전대를 못 보고 뒤판만 본다. 실제 충전대에도 흔한 모양이다.
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

MODEL = '''<?xml version="1.0"?>
<!-- make_aruco_dock.py 가 만든 ArUco dock ({board}, marker {ids}).
     원점 = 바닥에서 충전 접점 면의 가운데, +x = 방 쪽 (로봇이 다가오는 쪽), z = 위.
     충전대(높이 {base_h} m)와 그 뒤 {offset} m 의 뒤판, 뒤판 앞면에 marker 판. 월드에서는 +x 가 방을 보게 놓는다 -->
<sdf version="1.6">
  <model name="{model}">
    <static>true</static>
    <link name="body">
      <collision name="base_collision">
        <pose>{base_cx:.4f} 0 {base_cz:.4f} 0 0 0</pose>
        <geometry><box><size>{offset:.4f} {width:.4f} {base_h:.4f}</size></box></geometry>
      </collision>
      <visual name="base_visual">
        <pose>{base_cx:.4f} 0 {base_cz:.4f} 0 0 0</pose>
        <geometry><box><size>{offset:.4f} {width:.4f} {base_h:.4f}</size></box></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/DarkGrey</name></script></material>
      </visual>
      <visual name="contact_visual">
        <pose>0.002 0 {contact_z:.4f} 0 0 0</pose>
        <geometry><box><size>0.004 0.12 0.02</size></box></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Orange</name></script></material>
      </visual>
      <collision name="panel_collision">
        <pose>{panel_cx:.4f} 0 {panel_cz:.4f} 0 0 0</pose>
        <geometry><box><size>{panel_t:.4f} {width:.4f} {panel_h:.4f}</size></box></geometry>
      </collision>
      <visual name="panel_visual">
        <pose>{panel_cx:.4f} 0 {panel_cz:.4f} 0 0 0</pose>
        <geometry><box><size>{panel_t:.4f} {width:.4f} {panel_h:.4f}</size></box></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/DarkGrey</name></script></material>
      </visual>
      <visual name="board_visual">
        <pose>{plate_cx:.4f} {plate_cy:.4f} {plate_cz:.4f} 0 0 0</pose>
        <geometry><box><size>0.002 {plate_w:.4f} {plate_h:.4f}</size></box></geometry>
        <material>
          <script>
            <uri>model://{model}/materials/scripts</uri>
            <uri>model://{model}/materials/textures</uri>
            <name>ArucoDock/{name}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''

CONFIG = '''<?xml version="1.0"?>
<model>
  <name>{model}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>Charging dock with an ArUco board ({board}) behind the contact face (bt_mission, ARUCO_DOCKING.md)</description>
</model>
'''


def board_image(cfg, dictionary, px_per_m):
    """판 그림: 흰 바탕에 marker. 그림의 오른쪽 = D 의 +y, 위 = +z (로봇이 dock 을 바라본 모습)"""
    plate = cfg['plate']
    width = int(round((plate['y_max'] - plate['y_min']) * px_per_m))
    height = int(round((plate['z_max'] - plate['z_min']) * px_per_m))
    image = np.full((height, width), 255, np.uint8)
    for m in cfg['markers']:
        side = int(round(m['size'] * px_per_m))
        marker = cv2.aruco.generateImageMarker(dictionary, m['id'], side)
        u0 = int(round((m['y'] - m['size'] / 2 - plate['y_min']) * px_per_m))
        v0 = int(round((plate['z_max'] - (m['z'] + m['size'] / 2)) * px_per_m))
        image[v0:v0 + side, u0:u0 + side] = marker
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('boards', help='aruco_boards.yaml')
    parser.add_argument('board', help='board name in the yaml, e.g. dock_a')
    parser.add_argument('out', help='output PNG (for printing)')
    parser.add_argument('--px-per-m', type=float, default=4000.0)
    parser.add_argument('--gazebo-model', help='also write a Gazebo dock model to this directory')
    parser.add_argument('--base-height', type=float, default=0.05, help='charger block height [m]')
    args = parser.parse_args()

    with open(args.boards) as f:
        boards = yaml.safe_load(f)
    cfg = boards['boards'][args.board]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, boards['dictionary']))
    image = board_image(cfg, dictionary, args.px_per_m)
    cv2.imwrite(args.out, image)
    plate = cfg['plate']
    print(f"board '{args.board}': {len(cfg['markers'])} markers {[m['id'] for m in cfg['markers']]}, "
          f"plate {plate['y_max'] - plate['y_min']:.3f} x {plate['z_max'] - plate['z_min']:.3f} m -> {args.out} "
          f'({image.shape[1]} x {image.shape[0]} px, {args.px_per_m * 0.0254:.1f} dpi)')
    for m in cfg['markers']:
        print(f"  id {m['id']}: {m['size'] * 1000:.1f} mm, center y {m['y']:+.3f} z {m['z']:.3f} m")

    if args.gazebo_model:
        model = os.path.basename(os.path.normpath(args.gazebo_model))
        texture = f'aruco_{args.board}.png'
        os.makedirs(os.path.join(args.gazebo_model, 'materials', 'scripts'), exist_ok=True)
        os.makedirs(os.path.join(args.gazebo_model, 'materials', 'textures'), exist_ok=True)
        cv2.imwrite(os.path.join(args.gazebo_model, 'materials', 'textures', texture),
                    board_image(cfg, dictionary, 2000.0))
        with open(os.path.join(args.gazebo_model, 'materials', 'scripts', 'aruco_dock.material'), 'w') as f:
            f.write(MATERIAL.format(name=args.board, texture=texture))
        offset = cfg['board_offset']
        panel_t, panel_h = 0.02, plate['z_max'] + 0.04
        width = max(0.40, plate['y_max'] - plate['y_min'] + 0.04)
        with open(os.path.join(args.gazebo_model, 'model.sdf'), 'w') as f:
            f.write(MODEL.format(
                model=model, name=args.board, board=args.board, ids=[m['id'] for m in cfg['markers']],
                offset=offset, width=width, base_h=args.base_height, base_cx=-offset / 2, base_cz=args.base_height / 2,
                contact_z=args.base_height * 0.6, panel_t=panel_t, panel_h=panel_h,
                # 판 그림(두께 2 mm)의 앞면이 정확히 x = -board_offset 에 오게, 뒤판은 그 뒤에
                panel_cx=-offset - 0.002 - panel_t / 2, panel_cz=panel_h / 2,
                plate_cx=-offset - 0.001, plate_cy=(plate['y_min'] + plate['y_max']) / 2,
                plate_cz=(plate['z_min'] + plate['z_max']) / 2,
                plate_w=plate['y_max'] - plate['y_min'], plate_h=plate['z_max'] - plate['z_min']))
        with open(os.path.join(args.gazebo_model, 'model.config'), 'w') as f:
            f.write(CONFIG.format(model=model, board=args.board))
        print(f'Gazebo model {model} -> {args.gazebo_model}')


if __name__ == '__main__':
    main()
