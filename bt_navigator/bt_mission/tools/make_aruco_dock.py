#!/usr/bin/env python3
"""
config/aruco_docks.yaml 의 dock 하나로 인쇄용 PNG (marker + 흰 여백) 와, 원하면 Gazebo dock 모델을 만든다.

  python3 make_aruco_dock.py ../config/aruco_docks.yaml dock_a dock_a.png
  python3 make_aruco_dock.py ../config/aruco_docks.yaml dock_a dock_a.png \\
      --gazebo-model ../../../simple_line/simple_line_bringup/models/aruco_dock

인쇄: PNG 는 --px-per-m (기본 4000 = 101.6 dpi) 로 그린다. 인쇄한 뒤 marker 한 변(검은 테두리 바깥)을 자로 재서
yaml 의 size 와 다르면 고친다 (또는 aruco_detector.py 의 marker_scale). 휘지 않게 단단한 판(폼보드, 아크릴)에
붙이고, 반사가 적은 무광 용지를 쓴다. 흰 여백(quiet zone)은 marker 한 변의 1/4 이상 남긴다.

Gazebo 모델 (원점 = dock 좌표계 D 의 원점: 바닥, 충전 접점 면의 가운데, +x = 방 쪽):
  - 충전대: 높이 0.05 m 의 낮은 상자 (x: -offset ~ 0). 앞면에 접점 판(주황)
  - 뒤판: x = -offset 에 세운 판. 앞면에 marker 그림
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
  <description>Charging dock with an ArUco marker ({board}) behind the contact face (bt_mission, DOCKING.md)</description>
</model>
'''


def marker_image(dictionary, marker_id, side):
    if hasattr(cv2.aruco, 'generateImageMarker'):          # OpenCV 4.7+
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    return cv2.aruco.drawMarker(dictionary, marker_id, side)


def plate(cfg):
    """흰 판 (marker + 여백 1/4): D 의 y, z 범위"""
    half = cfg['size'] * 0.75
    lateral = cfg.get('lateral', 0.0)
    return {'y_min': lateral - half, 'y_max': lateral + half,
            'z_min': cfg['height'] - half, 'z_max': cfg['height'] + half}


def board_image(cfg, dictionary, px_per_m):
    """판 그림: 흰 바탕에 marker. 그림의 오른쪽 = D 의 +y, 위 = +z (로봇이 dock 을 바라본 모습)"""
    p = plate(cfg)
    width = int(round((p['y_max'] - p['y_min']) * px_per_m))
    height = int(round((p['z_max'] - p['z_min']) * px_per_m))
    image = np.full((height, width), 255, np.uint8)
    side = int(round(cfg['size'] * px_per_m))
    marker = marker_image(dictionary, cfg['marker_id'], side)
    u0 = (width - side) // 2
    v0 = (height - side) // 2
    image[v0:v0 + side, u0:u0 + side] = marker
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('docks', help='aruco_docks.yaml')
    parser.add_argument('dock', help='dock name in the yaml, e.g. dock_a')
    parser.add_argument('out', help='output PNG (for printing)')
    parser.add_argument('--px-per-m', type=float, default=4000.0)
    parser.add_argument('--gazebo-model', help='also write a Gazebo dock model to this directory')
    parser.add_argument('--base-height', type=float, default=0.05, help='charger block height [m]')
    args = parser.parse_args()

    with open(args.docks) as f:
        docks = yaml.safe_load(f)
    cfg = docks['docks'][args.dock]
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, docks['dictionary']))
    image = board_image(cfg, dictionary, args.px_per_m)
    cv2.imwrite(args.out, image)
    p = plate(cfg)
    print(f"dock '{args.dock}': marker {cfg['marker_id']} ({docks['dictionary']}), {cfg['size'] * 1000:.1f} mm, "
          f"center {cfg['height']:.3f} m high, {cfg['offset']:.3f} m behind the contact face -> {args.out} "
          f'({image.shape[1]} x {image.shape[0]} px, {args.px_per_m * 0.0254:.1f} dpi)')

    if args.gazebo_model:
        model = os.path.basename(os.path.normpath(args.gazebo_model))
        texture = f'aruco_{args.dock}.png'
        os.makedirs(os.path.join(args.gazebo_model, 'materials', 'scripts'), exist_ok=True)
        os.makedirs(os.path.join(args.gazebo_model, 'materials', 'textures'), exist_ok=True)
        cv2.imwrite(os.path.join(args.gazebo_model, 'materials', 'textures', texture),
                    board_image(cfg, dictionary, 4000.0))
        with open(os.path.join(args.gazebo_model, 'materials', 'scripts', 'aruco_dock.material'), 'w') as f:
            f.write(MATERIAL.format(name=args.dock, texture=texture))
        offset = cfg['offset']
        panel_t, panel_h = 0.02, p['z_max'] + 0.04
        width = max(0.40, p['y_max'] - p['y_min'] + 0.04)
        with open(os.path.join(args.gazebo_model, 'model.sdf'), 'w') as f:
            f.write(MODEL.format(
                model=model, name=args.dock, board=args.dock, ids=cfg['marker_id'],
                offset=offset, width=width, base_h=args.base_height, base_cx=-offset / 2, base_cz=args.base_height / 2,
                contact_z=args.base_height * 0.6, panel_t=panel_t, panel_h=panel_h,
                # 판 그림(두께 2 mm)의 앞면이 정확히 x = -board_offset 에 오게, 뒤판은 그 뒤에
                panel_cx=-offset - 0.002 - panel_t / 2, panel_cz=panel_h / 2,
                plate_cx=-offset - 0.001, plate_cy=(p['y_min'] + p['y_max']) / 2,
                plate_cz=(p['z_min'] + p['z_max']) / 2,
                plate_w=p['y_max'] - p['y_min'], plate_h=p['z_max'] - p['z_min']))
        with open(os.path.join(args.gazebo_model, 'model.config'), 'w') as f:
            f.write(CONFIG.format(model=model, board=args.dock))
        print(f'Gazebo model {model} -> {args.gazebo_model}')


if __name__ == '__main__':
    main()
