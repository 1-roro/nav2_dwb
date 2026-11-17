#!/usr/bin/env python3
"""DOCKING_STUDY.md 2장의 숫자: 합성 영상 한 장에서 검출의 각 단계 (모서리, IPPE 두 해, MAP, 공분산), 거리별 σ.

  python3 ex_detector.py

OpenCV (aruco 포함), numpy, pytest, yaml 이 있는 Python 이면 된다 (ROS 는 필요 없다)."""
import importlib.util
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.join(HERE, '..', '..', 'test', 'test_aruco_detector.py')
spec = importlib.util.spec_from_file_location('T', TEST)
T = importlib.util.module_from_spec(spec); spec.loader.exec_module(T)
ad = T.ad
np.set_printoptions(precision=4, suppress=True)
dock = T.ALL_DOCKS['dock_a']; marker = dock.markers[0]
camera = ad.Camera(T.K, np.zeros(5), T.R_BASE_CAM, T.T_BASE_CAM)
scene = (marker, dock, None, ad.MarkerDetector(ad.aruco_dictionary(T.DICTIONARY)), camera)
print('K =\n', T.K)
print('marker corners in D (m):\n', marker.corners_D)

robot = (0.75, 0.04, math.pi + math.radians(3.0))      # D 에서 로봇: 0.75 m 앞, 4 cm 옆, 3 도 틀어짐
truth = T.dock_in_base(robot)
print('robot in D', robot[0], robot[1], math.degrees(robot[2]))
print('truth dock in base x y yaw(deg)', truth[0], truth[1], math.degrees(truth[2]))
# 손으로 투영: 모서리 0 을 base -> camera -> pixel
c0_base = ad.rot_z(truth[2]) @ marker.corners_D[0] + np.array([truth[0], truth[1], 0])
c0_cam = camera.R_cam_base @ c0_base + camera.t_cam_base
print('corner0 in base', c0_base, 'in camera', c0_cam)
u = T.K[0, 0] * c0_cam[0] / c0_cam[2] + T.K[0, 2]; v = T.K[1, 1] * c0_cam[1] / c0_cam[2] + T.K[1, 2]
print('corner0 pixel by hand', u, v)
proj = ad.project_dock(truth, marker, camera)
print('all corners projected (true)\n', proj)

image = T.render(scene, robot, seed=7)
found = scene[3].detect(image)
corners = found[0]
print('detected corners\n', corners)
print('pixel error vs true projection\n', corners - proj)
ok, side = ad.usable(corners, T.W, T.H, 16.0)
print('usable', ok, 'side px', side, 'border px', ad.border_distance(corners, T.W, T.H))
sols = ad.ippe(corners, marker, camera)
for i, (R, t, e) in enumerate(sols):
    p6 = ad.dock6_from_marker(R, t, marker, camera)
    print(f'IPPE {i}: reproj rms {e:.3f} px, t_cam {t}, dist {np.linalg.norm(t):.4f}')
    print('   dock6 in base: x %.4f y %.4f yaw %.2f deg z %.4f roll %.2f pitch %.2f' % (p6[0], p6[1], math.degrees(p6[2]), p6[3], math.degrees(p6[4]), math.degrees(p6[5])))
    out = ad.refine_map(p6, corners, marker.corners_D, camera, ad.Prior(), 0.5)
    p, cost, rms, J = out
    print('   MAP: x %.4f y %.4f yaw %.3f deg z %.4f roll %.3f pitch %.3f | cost %.3f rms %.3f px' % (p[0], p[1], math.degrees(p[2]), p[3], math.degrees(p[4]), math.degrees(p[5]), cost, rms))
res = ad.estimate(corners, marker, camera)
for q, cov, rms in res['hypotheses']:
    s = np.sqrt(np.diag(cov))
    print('hyp: q', q, 'yaw deg', math.degrees(q[2]), 'sigma x %.2f mm y %.2f mm yaw %.3f deg' % (s[0]*1000, s[1]*1000, math.degrees(s[2])))
    err = q - truth; err[2] = ad.angle(err[2])
    print('   error vs truth: x %.2f mm y %.2f mm yaw %.3f deg' % (err[0]*1000, err[1]*1000, math.degrees(err[2])))
    print('   lateral error at docked pose %.2f mm' % (T.lateral_error_at_docked(q, truth) * 1000))
print('ambiguity', res['ambiguity'], 'tilt deg', math.degrees(res['tilt']), 'height mm', res['height']*1000, 'distance', res['distance'])
# 같은 자세에서 거리별 시그마
print('--- sigma vs distance (robot straight in front)')
for d in (1.0, 0.75, 0.56, 0.40, 0.31):
    r = (d, 0.0, math.pi)
    img = T.render(scene, r, seed=3)
    f = scene[3].detect(img)
    if 0 not in f:
        print(d, 'not found'); continue
    e = ad.estimate(f[0], marker, camera)
    q, cov, rms = e['hypotheses'][0]
    s = np.sqrt(np.diag(cov))
    side = ad.usable(f[0], T.W, T.H, 16)[1]
    tr = T.dock_in_base(r)
    print(f'{d:.2f} m: side {side:.0f} px, sigma x {s[0]*1000:.2f} y {s[1]*1000:.2f} mm yaw {math.degrees(s[2]):.3f} deg, hyps {len(e["hypotheses"])}, '
          f'err yaw {math.degrees(ad.angle(q[2]-tr[2])):+.3f} lat@dock {T.lateral_error_at_docked(q, tr)*1000:+.2f} mm')
