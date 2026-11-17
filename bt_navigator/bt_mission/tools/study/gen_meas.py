#!/usr/bin/env python3
"""DOCKING_STUDY.md 3-6 의 입력: staging 에서 서서 8장, 다가가며 100장의 합성 영상을 검출해 odom(=창고)에서 본 dock 의
측정과 공분산을 meas.txt 에 적는다. 그 다음 ex_filter (C++) 가 읽는다.

  python3 gen_meas.py && g++ -std=c++17 -I../../include ex_filter.cpp -o ex_filter && ./ex_filter filter && ./ex_filter step"""
import importlib.util
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.join(HERE, '..', '..', 'test', 'test_aruco_detector.py')
spec = importlib.util.spec_from_file_location('T', TEST)
T = importlib.util.module_from_spec(spec); spec.loader.exec_module(T)
ad = T.ad
dock = T.ALL_DOCKS['dock_a']; marker = dock.markers[0]
camera = ad.Camera(T.K, np.zeros(5), T.R_BASE_CAM, T.T_BASE_CAM)
scene = (marker, dock, None, ad.MarkerDetector(ad.aruco_dictionary(T.DICTIONARY)), camera)
DOCK = np.array([4.55, 1.0, math.pi])


def world_of(robot_D):
    x, y, yaw = robot_D; c, s = math.cos(DOCK[2]), math.sin(DOCK[2])
    return np.array([DOCK[0] + c * x - s * y, DOCK[1] + s * x + c * y, ad.angle(DOCK[2] + yaw)])


out = open('meas.txt', 'w')
seed = 0


def frame(robot_D, phase):
    global seed
    seed += 1
    img = T.render(scene, robot_D, seed=seed)
    f = scene[3].detect(img)
    if 0 not in f:
        return
    e = ad.estimate(f[0], marker, camera)
    rw = world_of(robot_D)
    c, s = math.cos(rw[2]), math.sin(rw[2])
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    hs = []
    for q, cov, rms in e['hypotheses']:
        z = np.array([rw[0] + c * q[0] - s * q[1], rw[1] + s * q[0] + c * q[1], ad.angle(rw[2] + q[2])])
        C = R @ cov @ R.T
        hs.append(' '.join(f'{v:.9g}' for v in list(z) + list(C.reshape(-1))))
    out.write(f'{phase} {rw[0]:.6f} {rw[1]:.6f} {rw[2]:.6f} {robot_D[0]:.4f} {len(hs)} ' + ' '.join(hs) + '\n')


# find_marker: 서서 8장 (staging: 0.75 m, 옆 4 cm, 3 도)
start = (0.75, 0.04, math.pi + math.radians(3.0))
for k in range(8):
    frame(start, 'look')
# approach: 15 Hz, 0.75 -> 0.56 m, 옆과 방향을 먼저 줄인다 (대략의 길)
n = 40
for k in range(1, n + 1):
    a = k / n
    x = 0.75 - 0.19 * a
    y = 0.04 * max(0.0, 1 - 1.6 * a)
    yaw = math.pi + math.radians(3.0) * max(0.0, 1 - 1.4 * a)
    frame((x, y, yaw), 'move')
# insert: 0.56 -> 0.31
for k in range(1, 61):
    a = k / 60
    frame((0.56 - 0.25 * a, 0.0, math.pi), 'move')
out.close()
print('ok', sum(1 for _ in open('meas.txt')))
