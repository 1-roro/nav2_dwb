#!/usr/bin/env python3
"""MECANUM_STUDY.md 의 숫자를 낸다 (numpy 만 있으면 된다. ROS 없이):

  python3 ex_mecanum.py

로봇 치수는 simple_line_bringup/robots/make_mecanum.py 에서 읽는다 (그 파일이 없으면 같은 값을 쓴다).
좌표: x = 로봇의 앞, y = 왼쪽, wz > 0 = 위에서 볼 때 반시계. 바퀴 순서: 왼앞, 오른앞, 왼뒤, 오른뒤.
바퀴 각속도 ω > 0 = 그 바퀴가 로봇을 앞으로 굴리는 쪽.
"""
import importlib.util
import math
import os

import numpy as np

np.set_printoptions(precision=4, suppress=True)
HERE = os.path.dirname(os.path.abspath(__file__))
GEN = os.path.join(HERE, '..', '..', '..', '..', 'simple_line', 'simple_line_bringup', 'robots', 'make_mecanum.py')
NAMES = ['FL', 'FR', 'RL', 'RR']

if os.path.exists(GEN):
    spec = importlib.util.spec_from_file_location('make_mecanum', GEN)
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    R, LX, LY = gen.WHEEL_R, gen.WHEEL_X, gen.WHEEL_Y
    WHEELS = [(sx * LX, sy * LY, tilt) for _, sx, sy, tilt in gen.WHEELS]
else:
    R, LX, LY = 0.04, 0.11, 0.1325
    WHEELS = [(LX, LY, -1), (LX, -LY, 1), (-LX, LY, 1), (-LX, -LY, -1)]
K = LX + LY


def section(title):
    print('\n=== ' + title)


def jacobian(wheels, r=R):
    """바닥에 닿은 롤러의 축 a = (1, tilt)/√2 에서 역기구학 행렬 H (ω = H v) 를 유도한다.
    롤러는 자기 축으로 자유롭게 돈다 -> 바퀴가 닿은 점은 a 에 수직으로는 마음대로 미끄러지고, a 방향으로는 못 미끄러진다.
    그래서 (바퀴 자리의 속도) · a = (바퀴가 굴러 내는 속도 r ω x̂) · a  ->  ω = (v_i · a) / (r a_x)"""
    rows = []
    for x, y, tilt in wheels:
        a = np.array([1.0, tilt]) / math.sqrt(2.0)
        # 바퀴 자리의 속도 v_i = (vx - wz y, vy + wz x) 를 (vx, vy, wz) 의 선형식으로
        rows.append(np.array([a[0], a[1], -y * a[0] + x * a[1]]) / (r * a[0]))
    return np.array(rows)


def wheels(H, v):
    return H @ np.asarray(v, float)


def show(label, v, H):
    w = wheels(H, v)
    rpm = w * 60 / (2 * math.pi)
    print(f'{label:28s} v=({v[0]:+.3f}, {v[1]:+.3f}, {v[2]:+.4f})  ω[rad/s] ' +
          '  '.join(f'{n} {x:+7.3f}' for n, x in zip(NAMES, w)) +
          '   rpm ' + ' '.join(f'{x:+6.1f}' for x in rpm))
    return w


H = jacobian(WHEELS)

section('치수와 역기구학 행렬')
print(f'r = {R} m, lx = {LX} m, ly = {LY} m, lx + ly = {K:.4f} m')
for n, (x, y, tilt) in zip(NAMES, WHEELS):
    print(f'  {n}: 자리 ({x:+.4f}, {y:+.4f}), 바닥 롤러의 축 (1, {tilt:+d})/√2')
print('H (ω = H [vx, vy, wz]):')
print(H)
print('r·H (= [1, ∓1, ∓(lx+ly)] 꼴):')
print(R * H)
ref = np.array([[1, -1, -K], [1, 1, K], [1, 1, -K], [1, -1, K]]) / R
print('wheel_joint_publisher.py 의 식과 같은가:', np.allclose(H, ref))

section('기본 움직임 (바퀴 속도)')
basic = [('앞으로 0.10 m/s', (0.10, 0, 0)),
         ('왼쪽으로 0.10 m/s', (0, 0.10, 0)),
         ('반시계로 0.50 rad/s', (0, 0, 0.50)),
         ('왼앞 대각선 (0.1, 0.1)', (0.10, 0.10, 0)),
         ('앞 0.1 + 반시계 0.5', (0.10, 0, 0.50)),
         ('옆 0.1 + 반시계 0.5', (0, 0.10, 0.50))]
for label, v in basic:
    show(label, v, H)

section('겹치기: 도킹 예제 명령 (DOCKING_STUDY 4-4) 을 셋으로 나눠 더하기')
cmd = (0.045, 0.005, -0.0393)
parts = [('vx 만', (cmd[0], 0, 0)), ('vy 만', (0, cmd[1], 0)), ('wz 만', (0, 0, cmd[2]))]
total = np.zeros(4)
for label, v in parts:
    total += show(label, v, H)
w_cmd = show('합친 명령', cmd, H)
print('세 부분의 합 == 합친 명령:', np.allclose(total, w_cmd))

section('힘: 바퀴마다 같은 토크 τ = 0.1 N·m 일 때 몸체가 받는 힘 (F = Hᵀ τ)')
tau = 0.1
print(f'바퀴 하나의 힘 = (τ/r)(1, tilt) : 앞뒤 {tau / R:.2f} N, 옆 ±{tau / R:.2f} N, 크기 {tau / R * math.sqrt(2):.2f} N (롤러 축 방향)')
for label, v in basic[:3]:
    s = np.sign(np.round(wheels(H, v), 9))
    F = H.T @ (tau * s)
    print(f'{label:28s} 토크 부호 {s}  ->  Fx {F[0]:+.2f} N, Fy {F[1]:+.2f} N, Mz {F[2]:+.3f} N·m')
print('일률이 같다: Σ τ_i ω_i = F · v  (같은 행렬의 전치)')
v = np.array(cmd)
tq = np.array([0.05, -0.02, 0.08, 0.01])
print(f'  예: τ={tq}, v={v}:  Σ τ ω = {tq @ (H @ v):.6f},  (Hᵀ τ)·v = {(H.T @ tq) @ v:.6f}')

section('정기구학 (바퀴 -> 몸체, odom) 과 여분의 한 방향')
Hp = np.linalg.pinv(H)
print('H⁺ (v = H⁺ ω):')
print(Hp)
print('r/4 ·[1 1 1 1], r/4 ·[-1 1 1 -1], r/(4(lx+ly)) ·[-1 1 -1 1] 와 같은가:',
      np.allclose(Hp, np.array([[1, 1, 1, 1], [-1, 1, 1, -1], [-1 / K, 1 / K, -1 / K, 1 / K]]) * R / 4))
U, S, Vt = np.linalg.svd(H.T)
n = Vt[-1] / Vt[-1][np.argmax(np.abs(Vt[-1]))]
print('H 의 열들에 수직인 바퀴 속도 (어떤 몸체 속도로도 만들 수 없다):', n, ' 특이값', S)
w_meas = w_cmd + 0.3 * n
print('엔코더 = 명령 바퀴 속도 + 0.3·n :', w_meas)
print('  H⁺ 로 몸체 속도:', Hp @ w_meas, ' (명령과 같다: n 성분은 버려진다)')
print('  남는 것 (= 서로 싸운 양) :', w_meas - H @ (Hp @ w_meas))

section('마찰로 풀어 보기: 바퀴 속도를 정했을 때 몸체는 어떻게 움직이나 (점성 마찰 모형)')
# 바퀴 i 가 닿은 점이 롤러 축 방향으로 미끄러지는 속도 s_i = r a_x (H_i v - ω_i).
# 바닥은 그 반대로 c_i s_i 의 힘을 롤러 축 방향으로 준다 (롤러에 수직으로는 힘이 없다).
# 몸체: m dv/dt = 그 힘들의 합 (힘, 돌림힘). 시간이 지나면 미끄러짐의 가중 제곱합이 가장 작은 v 에 선다.
mass, inertia = 8.0, 8.0 * (0.32 ** 2 + 0.22 ** 2) / 12


def settle(w, c, t_end=1.0, dt=1e-4):
    v = np.zeros(3)
    a_x = 1 / math.sqrt(2)
    for _ in range(int(t_end / dt)):
        slip = R * a_x * (H @ v - w)                     # 바퀴마다 롤러 축 방향 미끄러짐 [m/s]
        f = -(H.T * R * a_x) @ (c * slip)                # (Fx, Fy, Mz)
        v = v + dt * f / np.array([mass, mass, inertia])
    return v, R * a_x * (H @ v - w)


for label, w, c in [('맞는 바퀴 속도 (도킹 예제)', w_cmd, np.full(4, 200.0)),
                    ('+ 싸우는 성분 0.3·n', w_cmd + 0.3 * n, np.full(4, 200.0)),
                    ('+ 싸우는 성분, 앞바퀴가 가벼움', w_cmd + 0.3 * n, np.array([100.0, 100.0, 300.0, 300.0]))]:
    v, slip = settle(w, c)
    Cm = np.diag(c)
    v_ls = np.linalg.solve(H.T @ Cm @ H, H.T @ Cm @ w)
    print(f'{label:30s} 몸체 {v}  (가중 최소제곱 {v_ls}),  롤러 축 방향 미끄러짐 [mm/s] {slip * 1000}')

section('잘못 끼운 바퀴 (O 배치: 롤러가 모두 거울상)')
Hw = jacobian([(x, y, -t) for x, y, t in WHEELS])
print('r·H_wrong:')
print(R * Hw)
print(f'회전 열의 크기 |ly - lx| = {abs(LY - LX):.4f} m  (바른 배치는 lx + ly = {K:.4f} m)')
print('특이값 바른 / 잘못:', np.linalg.svd(H, compute_uv=False), np.linalg.svd(Hw, compute_uv=False))
print(f'조건수 바른 / 잘못: {np.linalg.cond(H):.1f} / {np.linalg.cond(Hw):.1f}')
show('잘못: 반시계 0.5 rad/s', (0, 0, 0.5), Hw)
sq = jacobian([(0.12, 0.12, -t) for _, _, t in WHEELS])
print('정사각형 (lx = ly = 0.12) 에서 잘못 끼우면 rank =', np.linalg.matrix_rank(sq), '(회전을 만들 수 없다)')

section('바퀴 속도의 한계와 줄이는 방법')
w_max = 0.26 / R
print(f'바퀴 최고 ω = 0.26 / r = {w_max:.2f} rad/s ({w_max * 60 / 2 / math.pi:.1f} rpm)')
print('바른 배치에서 바퀴 넷 중 가장 빠른 것 = (|vx| + |vy| + (lx+ly)|wz|) / r')
nav = np.array([0.26, 0.15, 1.0])
w = show('Nav2 최고값을 한꺼번에', nav, H)
need = (abs(nav[0]) + abs(nav[1]) + K * abs(nav[2])) / R
print(f'  가장 빠른 바퀴 {need:.2f} rad/s = 한계의 {need / w_max:.2f} 배')
s = w_max / np.max(np.abs(w))
w_s = w * s
show('  (가) 넷을 같은 비율로 줄임', tuple(nav * s), H)
v_s = Hp @ w_s
w_c = np.clip(w, -w_max, w_max)
v_c = Hp @ w_c
print(f'  (가) 같은 비율 {s:.3f}: 몸체 {v_s},  방향 각 {math.degrees(math.atan2(v_s[1], v_s[0])):.1f}°, '
      f'회전/직진 비 {v_s[2] / math.hypot(v_s[0], v_s[1]):.3f}')
print(f'  (나) 바퀴마다 자름: 바퀴 {w_c},  몸체 {v_c},  방향 각 {math.degrees(math.atan2(v_c[1], v_c[0])):.1f}°, '
      f'회전/직진 비 {v_c[2] / math.hypot(v_c[0], v_c[1]):.3f}')
print(f'  원래 명령: 방향 각 {math.degrees(math.atan2(nav[1], nav[0])):.1f}°, 회전/직진 비 {nav[2] / math.hypot(nav[0], nav[1]):.3f}')
print(f'  (나) 에서 서로 싸운 양: {w_c - H @ v_c}')
for ang in (0, 45, 90):
    a = math.radians(ang)
    vmax = R * w_max / (abs(math.cos(a)) + abs(math.sin(a)))
    print(f'  회전 없이 {ang:2d}° 방향 최고 속도 = {vmax:.3f} m/s')

section('견인력 (밀 수 있는 힘): 바퀴 하나가 미끄러지기 직전')
mass, g, mu = 8.0, 9.81, 0.6
N = mass * g / 4
print(f'질량 {mass} kg, 바퀴 하나의 수직항력 N = {N:.2f} N, 마찰계수 μ = {mu} (가정)')
print(f'  보통 바퀴: 앞으로 최대 μN = {mu * N:.2f} N')
print(f'  메카넘: 힘은 롤러 축 방향이라 크기 ≤ μN, 앞 성분 ≤ μN·cos45° = {mu * N * math.cos(math.pi / 4):.2f} N '
      f'(보통 바퀴의 {math.cos(math.pi / 4):.0%})')
