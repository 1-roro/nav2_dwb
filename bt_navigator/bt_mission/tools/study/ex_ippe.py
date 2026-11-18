#!/usr/bin/env python3
"""DOCKING_BIG_PICTURE.md 7 ~ 8장의 숫자: 정사각형 하나의 PnP(IPPE) 두 해, 거리별로 얼마나 헷갈리나, 방향이 왜 약한가.

  python3 ex_ippe.py          # OpenCV (aruco 포함), numpy, pytest, yaml 이 있는 Python (ROS 없이)

합성 영상(test_aruco_detector.py 의 render, RealSense D455 1280 x 720) 을 실제 detector(aruco_detector.py) 로 본다.
로봇은 dock 정면 거리 d 에서 10° 돌아 서 있다. 거리마다 16장.
  - IPPE 의 두 해 가운데 재투영 오차가 작은 것을 고르면 틀린 해를 고르는 비율
  - 그때 방향 오차: 고른 해, 맞는 해, 이 프로젝트의 MAP (바닥 사전분포)
  - 방향을 일부러 틀리게 두고 나머지를 가장 잘 맞췄을 때의 재투영 오차 (멀면 작다 = 방향이 영상에 흔적을 덜 남긴다)
"""
import importlib.util
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.join(HERE, '..', '..', 'test', 'test_aruco_detector.py')
spec = importlib.util.spec_from_file_location('T', TEST)
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)
ad = T.ad
marker = T.ALL_DOCKS['dock_a'].markers[0]
camera = ad.Camera(T.K, np.zeros(5), T.R_BASE_CAM, T.T_BASE_CAM)
det = ad.MarkerDetector(ad.aruco_dictionary(T.DICTIONARY))
scene = (marker, T.ALL_DOCKS['dock_a'], None, det, camera)
SIG = 0.5


def resid(p, u):
    pr = ad.project_dock6(p, marker, camera)
    return None if pr is None else ((pr - u) / SIG).reshape(-1)


def fit_fixed_yaw(p0, yaw, u, iters=40):
    """방향(yaw)을 고정하고 나머지 5자유도 (x, y, z, roll, pitch) 를 Levenberg-Marquardt 로 맞춘다"""
    p = np.array(p0, float)
    p[2] = yaw
    free = [0, 1, 3, 4, 5]
    lam = 1e-3
    r = resid(p, u)
    cost = r @ r
    for _ in range(iters):
        J = np.zeros((len(r), len(free)))
        for j, i in enumerate(free):
            d = np.zeros(6)
            d[i] = 1e-6
            J[:, j] = (resid(p + d, u) - r) / 1e-6
        H, g = J.T @ J, J.T @ r
        while True:
            step = -np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-12), g)
            q = p.copy()
            q[free] += step
            rq = resid(q, u)
            if rq is not None and rq @ rq < cost:
                p, r, cost = q, rq, rq @ rq
                lam = max(lam / 3, 1e-9)
                break
            lam *= 4
            if lam > 1e8:
                return p, cost
        if np.abs(step).max() < 1e-9:
            break
    return p, cost


def yaw_profile(dist, view_deg=5.0, span=12.0, step=2.0):
    truth = T.dock_in_base((dist, 0.0, math.pi + math.radians(view_deg)))
    p_true = np.array([truth[0], truth[1], truth[2], 0, 0, 0])
    u = ad.project_dock6(p_true, marker, camera)
    out, prev = [], p_true.copy()
    for dy in np.arange(-span, span + 1e-9, step):
        best = min((fit_fixed_yaw(init, truth[2] + math.radians(dy), u) for init in (prev, p_true)), key=lambda x: x[1])
        prev = best[0]
        img = resid(best[0], u)
        out.append((dy, math.sqrt(img @ img * SIG ** 2 / 4)))
    return out


def main():
    print('dist  rms(맞는 해)  rms(틀린 해)  오차 작은 해가 틀림   |방향 오차| 중앙값: 고른 해 / MAP')
    wrong_cases, frames, two = [], 0, 0
    for dist in np.round(np.arange(1.20, 0.339, -0.05), 2):
        robot = (float(dist), 0.0, math.pi + math.radians(10.0))
        truth = T.dock_in_base(robot)
        rows = []
        for seed in range(16):
            found = det.detect(T.render(scene, robot, seed=100 + seed))
            if 0 not in found:
                continue
            c = found[0].reshape(-1, 2).astype(np.float64)
            sols = ad.ippe(c, marker, camera)
            q = [ad.dock6_from_marker(R, t, marker, camera) for R, t, e in sols]
            rms = [e for R, t, e in sols]
            d6 = [math.hypot(math.degrees(ad.angle(x[2] - truth[2])), math.degrees(x[5])) for x in q]
            pick, right = int(np.argmin(rms)), int(np.argmin(d6))
            e = ad.estimate(c, marker, camera)
            frames += 1
            two += len(e['hypotheses']) > 1
            yaw = lambda a: abs(math.degrees(ad.angle(a - truth[2])))  # noqa: E731
            rows.append((rms[right], rms[1 - right], pick != right, yaw(q[pick][2]), yaw(e['hypotheses'][0][0][2])))
            if pick != right:
                wrong_cases.append((yaw(q[pick][2]), yaw(q[right][2]), yaw(e['hypotheses'][0][0][2])))
        a = np.array(rows, float)
        print(f'{dist:.2f}  {np.median(a[:, 0]):10.3f}  {np.median(a[:, 1]):11.3f}  {a[:, 2].mean() * 100:14.0f} %'
              f'       {np.median(a[:, 3]):5.2f} / {np.median(a[:, 4]):5.2f}°')
    w = np.array(wrong_cases)
    print(f'\n{frames} 장. MAP 이 후보를 둘 낸 장: {two}')
    print(f'오차 작은 해가 틀린 {len(w)} 장의 방향 오차 중앙값: 고른 해 {np.median(w[:, 0]):.2f}°, 맞는 해 {np.median(w[:, 1]):.2f}°,'
          f' MAP {np.median(w[:, 2]):.2f}°')
    print('\n방향을 일부러 틀리게 두었을 때의 재투영 RMS [px] (나머지는 가장 잘 맞춤, 잡음 없는 모서리):')
    for dist in (1.0, 0.6, 0.35):
        prof = yaw_profile(dist)
        print(f'  {dist:.2f} m: ' + '  '.join(f'{dy:+.0f}°:{r:.2f}' for dy, r in prof if abs(dy) in (4, 10)))


if __name__ == '__main__':
    main()
