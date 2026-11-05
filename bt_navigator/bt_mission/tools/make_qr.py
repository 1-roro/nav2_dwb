#!/usr/bin/env python3
"""
dock 에 붙일 QR 그림을 만든다. 인쇄용 PNG 와, 원하면 Gazebo 의 dock 모델(QR 판이 붙은 상자)도 만든다.

  python3 make_qr.py DOCK_A dock_a.png --print-size 0.20
  python3 make_qr.py DOCK_A dock_a.png --print-size 0.20 --gazebo-model ../../simple_line/simple_line_bringup/models/qr_dock

QR 둘레에는 4칸 흰 여백(quiet zone)을 둔다. 여백이 없으면 잘 읽지 못한다.
qr_detector.py 의 qr_size 는 여백을 뺀 QR 한 변이다. 이 도구가 인쇄 크기에서 계산해 알려 주지만,
인쇄하면 크기가 조금 달라지므로 인쇄한 것을 자로 재서 넣는다.
"""

import argparse
import os

import cv2
import numpy as np

QUIET = 4      # 여백 [칸]

# emissive: 조명이 어두운 월드에서도 흰 칸이 희게 (인쇄한 종이처럼) 보이게
MATERIAL = '''material QrDock/{name}
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
        filtering none
      }}
    }}
  }}
}}
'''

# 모델 원점 = QR 판 앞면의 바닥 자리, +x = QR 앞으로 나오는 법선 (로봇이 다가오는 쪽). 몸통은 -x 쪽에 있다
MODEL = '''<?xml version="1.0"?>
<!-- make_qr.py 가 만든 dock: 회색 몸통 앞면에 QR 판 ({text}, 판 {plate:.3f} m, QR {code:.4f} m).
     원점 = QR 판 앞면의 바닥 자리, +x = QR 앞으로 나오는 법선. 월드에서는 앞면이 방을 보게 놓는다 -->
<sdf version="1.6">
  <model name="{model}">
    <static>true</static>
    <link name="body">
      <collision name="body_collision">
        <pose>-0.1 0 0.15 0 0 0</pose>
        <geometry><box><size>0.2 0.5 0.3</size></box></geometry>
      </collision>
      <visual name="body_visual">
        <pose>-0.1 0 0.15 0 0 0</pose>
        <geometry><box><size>0.2 0.5 0.3</size></box></geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/DarkGrey</name>
          </script>
        </material>
      </visual>
      <visual name="qr_visual">
        <pose>{half_thick:.4f} 0 {height:.3f} 0 0 0</pose>
        <geometry><box><size>{thick:.4f} {plate:.3f} {plate:.3f}</size></box></geometry>
        <material>
          <script>
            <uri>model://{model}/materials/scripts</uri>
            <uri>model://{model}/materials/textures</uri>
            <name>QrDock/{name}</name>
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
  <description>Charging dock with a QR plate ({text}) for the QR docking example (bt_mission, DOCKING.md)</description>
</model>
'''


def qr_image(text, module_px):
    """QR (여백 포함) 그림과 QR 칸 수"""
    code = cv2.QRCodeEncoder.create().encode(text)
    dark = np.where((code == 0).any(axis=1))[0]
    code = code[dark[0]:dark[-1] + 1, dark[0]:dark[-1] + 1]      # OpenCV 가 붙인 여백을 떼고 다시 붙인다
    modules = code.shape[0]
    big = cv2.resize(code, (modules * module_px, modules * module_px), interpolation=cv2.INTER_NEAREST)
    pad = QUIET * module_px
    return cv2.copyMakeBorder(big, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255), modules


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('text', help='QR content, e.g. DOCK_A (the dock task target)')
    parser.add_argument('out', help='output PNG')
    parser.add_argument('--print-size', type=float, default=0.20, help='printed size including the white margin [m]')
    parser.add_argument('--module-px', type=int, default=20, help='pixels per QR module in the PNG')
    parser.add_argument('--gazebo-model', help='also write a Gazebo dock model to this directory')
    parser.add_argument('--height', type=float, default=0.115, help='QR center height on the dock [m] (camera height)')
    args = parser.parse_args()

    image, modules = qr_image(args.text, args.module_px)
    cv2.imwrite(args.out, image)
    code_size = args.print_size * modules / (modules + 2 * QUIET)
    print(f"QR '{args.text}': {modules} modules + {QUIET} quiet modules each side -> {args.out}")
    print(f'printed at {args.print_size:.3f} m (with margin), the code is {code_size:.4f} m: '
          f'qr_detector.py qr_size:={code_size:.4f}')

    if args.gazebo_model:
        model = os.path.basename(os.path.normpath(args.gazebo_model))
        name = args.text.replace(' ', '_')
        texture = f'qr_{name}.png'
        os.makedirs(os.path.join(args.gazebo_model, 'materials', 'scripts'), exist_ok=True)
        os.makedirs(os.path.join(args.gazebo_model, 'materials', 'textures'), exist_ok=True)
        cv2.imwrite(os.path.join(args.gazebo_model, 'materials', 'textures', texture), image)
        with open(os.path.join(args.gazebo_model, 'materials', 'scripts', 'qr_dock.material'), 'w') as f:
            f.write(MATERIAL.format(name=name, texture=texture))
        with open(os.path.join(args.gazebo_model, 'model.sdf'), 'w') as f:
            f.write(MODEL.format(model=model, name=name, text=args.text, plate=args.print_size, code=code_size,
                                 thick=0.002, half_thick=0.001, height=args.height))
        with open(os.path.join(args.gazebo_model, 'model.config'), 'w') as f:
            f.write(CONFIG.format(model=model, text=args.text))
        print(f'Gazebo model {model} -> {args.gazebo_model}')


if __name__ == '__main__':
    main()
