#!/usr/bin/env python3
"""DOCKING_BIG_PICTURE.md 8-3 의 숫자: marker 하나(dock_a) 와 둘(dock_b, 60 mm 둘, 중심 간격 160 mm) 의 비교.

  python3 ex_two_markers.py [결과.json]      # OpenCV (aruco 포함), numpy, pytest, yaml (ROS 없이)

1. 평면 PnP 의 두 해: 보인 모서리 전부(하나면 4개, 둘이면 8개)를 한 평면으로 보고 IPPE 로 푼 두 해의 재투영 오차.
   평면 물체는 marker 가 몇 개든 해가 둘 나온다. 다만 물체가 크게 보이면 틀린 해의 오차가 커서 헷갈리지 않는다.
2. detector 가 실제로 하는 것 (estimate_dock): marker 마다 IPPE 두 해를 시작점으로 모서리 전부의 오차를 재고, 가장 좋은
   것과 다른 골짜기의 것 하나를 MAP 으로 다듬는다. 뒤집힌 시작점의 모서리 전부 오차, 마지막 후보 수.
3. 한 장의 방향 σ (공분산) 와 실제 방향 오차.
4. 1.0 m 에서 방향을 일부러 틀리게 두고 나머지를 가장 잘 맞췄을 때의 재투영 오차.
"""
import importlib.util
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.join(HERE, '..', '..', 'test', 'test_aruco_detector.py')
spec = importlib.util.spec_from_file_location('T', TEST)
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)
ad = T.ad
camera = ad.Camera(T.K, np.zeros(5), T.R_BASE_CAM, T.T_BASE_CAM)
det = ad.MarkerDetector(ad.aruco_dictionary(T.DICTIONARY))
DOCKS = {'one': T.ALL_DOCKS['dock_a'], 'two': T.ALL_DOCKS['dock_b']}
M = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])     # D -> 평면 좌표 (y_D, z_D, x_D)
SIG = 0.5


def observe(dock, robot, seed):
    scene = (dock.markers[0], dock, None, det, camera)
    found = det.detect(T.render(scene, robot, seed=seed, dock=dock))
    return [(m, found[m.marker_id].reshape(-1, 2).astype(np.float64)) for m in dock.markers
            if m.marker_id in found and ad.usable(found[m.marker_id], T.W, T.H, 16.0)[0]]


def planar_two_solutions(obs, truth):
    """모서리 전부를 한 평면 물체로 보고 IPPE: [(방향 오차 deg, 재투영 rms px)] 맞는 해가 먼저"""
    pts = np.vstack([m.corners_D for m, _ in obs])
    obj = (pts - np.array([pts[:, 0].mean(), 0.0, 0.0])) @ M.T
    img = np.vstack([c for _, c in obs])
    ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, img.reshape(-1, 1, 2), camera.K, camera.dist, flags=cv2.SOLVEPNP_IPPE)
    out = []
    for r, e in zip(rvecs, np.asarray(errs).reshape(-1)):
        R_base_D = camera.R_base_cam @ cv2.Rodrigues(r)[0] @ M
        out.append((math.degrees(ad.angle(math.atan2(R_base_D[1, 0], R_base_D[0, 0]) - truth[2])), float(e)))
    return sorted(out, key=lambda s: abs(s[0]))


def flipped_start_rms(obs):
    """estimate_dock 의 시작점들: marker 마다 IPPE 두 해. 모서리 전부에 대한 rms, 가장 좋은 것과 뒤집힌 것"""
    pts = np.vstack([m.corners_D for m, _ in obs])
    img = np.vstack([c for _, c in obs])
    rms = []
    for m, c in obs:
        for R, t, _ in ad.ippe(c, m, camera):
            pr = ad.project_points6(ad.dock6_from_marker(R, t, m, camera), pts, camera)
            if pr is not None:
                rms.append(math.sqrt(np.mean(np.sum((pr - img) ** 2, axis=1))))
    rms.sort()
    return rms[0], rms[-1]


def resid(p, points, u):
    pr = ad.project_points6(p, points, camera)
    return None if pr is None else ((pr - u) / SIG).reshape(-1)


def yaw_profile(dock, dist=1.0, view=5.0, span=10.0, step=0.5):
    truth = T.dock_in_base((dist, 0.0, math.pi + math.radians(view)))
    points = np.vstack([m.corners_D for m in dock.markers])
    p_true = np.array([truth[0], truth[1], truth[2], 0, 0, 0])
    u = ad.project_points6(p_true, points, camera)
    out, prev = [], p_true.copy()
    for dy in np.arange(-span, span + 1e-9, step):
        best = None
        for init in (prev, p_true):
            p = np.array(init, float)
            p[2] = truth[2] + math.radians(dy)
            free, lam = [0, 1, 3, 4, 5], 1e-3
            r = resid(p, points, u)
            cost = r @ r
            for _ in range(40):
                J = np.zeros((len(r), len(free)))
                for j, i in enumerate(free):
                    d = np.zeros(6)
                    d[i] = 1e-6
                    J[:, j] = (resid(p + d, points, u) - r) / 1e-6
                H, g = J.T @ J, J.T @ r
                moved = False
                while lam < 1e8:
                    step_ = -np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-12), g)
                    q = p.copy()
                    q[free] += step_
                    rq = resid(q, points, u)
                    if rq is not None and rq @ rq < cost:
                        p, r, cost, moved = q, rq, rq @ rq, True
                        lam = max(lam / 3, 1e-9)
                        break
                    lam *= 4
                if not moved or np.abs(step_).max() < 1e-9:
                    break
            if best is None or cost < best[1]:
                best = (p, cost)
        prev = best[0]
        out.append((float(dy), math.sqrt(best[1] * SIG ** 2 / len(points))))
    return out


def main():
    result = {'sweep': {}, 'profile': {}}
    print('dist | 평면 IPPE 틀린 해 rms (하나 / 둘) | 뒤집힌 시작점 rms (하나 / 둘) | 한 장 σ방향 (하나 / 둘) | |방향 오차| (하나 / 둘)')
    two_hyp = {'one': 0, 'two': 0}
    for dist in np.round(np.arange(1.20, 0.339, -0.05), 2):
        robot = (float(dist), 0.0, math.pi + math.radians(10.0))
        truth = T.dock_in_base(robot)
        row = {}
        for name, dock in DOCKS.items():
            vals = []
            for seed in range(16):
                obs = observe(dock, robot, 100 + seed)
                if len(obs) != len(dock.markers):
                    continue
                sols = planar_two_solutions(obs, truth)
                best_start, flipped = flipped_start_rms(obs)
                e = ad.estimate_dock(obs, camera)
                two_hyp[name] += len(e['hypotheses']) > 1
                q, cov, _ = e['hypotheses'][0]
                vals.append((sols[0][1], sols[1][1], best_start, flipped, math.degrees(math.sqrt(cov[2, 2])),
                             abs(math.degrees(ad.angle(q[2] - truth[2]))), float(sols[1][1] < sols[0][1]), abs(sols[1][0])))
            a = np.array(vals)
            row[name] = dict(n=len(vals), ippe_good=float(np.median(a[:, 0])), ippe_bad=float(np.median(a[:, 1])),
                             start_best=float(np.median(a[:, 2])), start_flipped=float(np.median(a[:, 3])),
                             sigma_yaw=float(np.median(a[:, 4])), yaw_err=float(np.median(a[:, 5])),
                             wrong_pick=float(np.mean(a[:, 6])), bad_yaw=float(np.median(a[:, 7])))
        result['sweep'][f'{dist:.2f}'] = row
        o, t = row['one'], row['two']
        print(f"{dist:.2f} | {o['ippe_bad']:7.3f} / {t['ippe_bad']:7.3f} | {o['start_flipped']:7.3f} / {t['start_flipped']:7.3f} |"
              f" {o['sigma_yaw']:6.2f}° / {t['sigma_yaw']:5.2f}° | {o['yaw_err']:5.2f}° / {t['yaw_err']:5.2f}°")
    print(f"MAP 이 후보를 둘 낸 장: 하나 {two_hyp['one']}, 둘 {two_hyp['two']} (각 288 장)")
    for name in DOCKS:
        w = [result['sweep'][k][name]['wrong_pick'] for k in result['sweep']]
        far = [result['sweep'][k][name]['wrong_pick'] for k in result['sweep'] if float(k) >= 0.95]
        bad = [result['sweep'][k][name]['bad_yaw'] for k in result['sweep'] if float(k) >= 0.95]
        print(f'평면 IPPE 에서 오차 작은 해가 틀린 비율 ({name}): 전체 {np.mean(w) * 100:.0f} %, 0.95 m 이상 {np.mean(far) * 100:.0f} %,'
              f' 그때 틀린 해의 방향 오차 중앙값 {np.median(bad):.1f}°')
    result['two_hyp'] = two_hyp
    for name, dock in DOCKS.items():
        prof = yaw_profile(dock)
        result['profile'][name] = prof
        print(f'1.0 m 에서 방향을 틀리게 두었을 때 rms ({name}): ' +
              '  '.join(f'{dy:+.0f}°:{r:.2f}' for dy, r in prof if abs(dy) in (2, 4, 10)))
    if len(sys.argv) > 1:
        json.dump(result, open(sys.argv[1], 'w'), indent=1)


if __name__ == '__main__':
    main()
