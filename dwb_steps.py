#!/usr/bin/env python3
"""
DWA를 4단계로 나눠 쌓아가며 비교하는 스크립트.

step0 은 dynamic_window_approach.py 와 동일한 동작(= 아직 DWA가 아님)이고,
step1~4 로 갈수록 원논문(Fox, Burgard, Thrun 1997)에 가까워진다.

  step0_baseline   속도공간 전역 샘플링 + 1스텝 예측 + 소프트 장애물 항
  step1_accel      가속도 한계로 샘플 범위를 좁힘        <- 여기서부터 'dynamic window'
  step2_rollout    sim_time 동안 궤적을 적분해 평가    <- 평가 함수 결함이 드러난다
  step3_progress   진척(progress) 항으로 정규화         <- 그 결함을 고친다
  step4_collision  충돌하는 궤적을 후보에서 탈락시킴    <- 하드 제약

step2 는 의도된 실패다.
  원본의 heading 항은 |목표까지 거리| * cos(방향오차) 로 거리에 비례한다.
  1스텝(0.1s)만 볼 때는 티가 안 나지만 1.7s 앞을 보게 되면
  "전진해서 거리를 줄이기"보다 "제자리 회전으로 정렬만 개선하기"가
  더 높은 점수를 받아 로봇이 출발조차 하지 않는다.
  step3 에서 정렬 대신 '목표까지 거리가 줄어든 양'을 보상해 해결한다.
  (nav2 DWB 의 GoalDist / PathDist critic 이 담당하는 역할)

실행:
  python3 dwa_steps.py            # 전체 비교, PNG 저장
  python3 dwa_steps.py --show     # 창으로 띄우기
  python3 dwa_steps.py --only step2_rollout
"""

import argparse

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# ---------------------------------------------------------------- 공통 설정

DT = 0.1                  # 제어 주기 [s]
T_MAX = 20.0              # 시뮬레이션 최대 시간 [s]

V_MAX, W_MAX = 1.0, 1.0           # 속도 상한 [m/s], [rad/s]
A_MAX, ALPHA_MAX = 1.0, 2.0       # 가속도 한계 [m/s^2], [rad/s^2]

N_V, N_W = 5, 5           # 속도공간 샘플 수 (v, w)
SIM_TIME = 1.7            # 궤적 예측 시간 [s] (nav2 DWB 기본값과 동일)

ROBOT_RADIUS = 0.25       # 충돌 판정 반경 [m]
SAFETY_MARGIN = 0.10      # 반경에 더해 띄울 여유 [m] (nav2 costmap의 inflation 역할)
CLEARANCE_REF = 2.0       # 정규화 기준 거리 [m]
GOAL_TOL = 0.2            # 목표 도달 판정 [m]

WEIGHTS = (0.8, 0.2, 0.5)  # (heading, clearance, velocity) - 원본과 동일
# clearance :실제 주행에서 장애물까지의 최소 거리.

OBSTACLES = np.array([[2.0, 2.0],
                      [2.5, 1.5],
                      [3.0, 3.0],
                      [3.0, 2.8],
                      [3.0, 2.86],
                      [4.0, 3.0]])
GOAL = np.array([5.0, 5.0])                     # Q. goal에 있어서 target theta goal도 설정은 어떻게 할까?
START = np.array([0.0, 0.0, 0.0])   # x, y, theta


# ---------------------------------------------------------------- 운동 모델

def motion(x, u, dt=DT):
    """차동구동 1스텝 전진. 원본과 달리 새 배열을 반환해 in-place 변형을 피한다."""
    v, w = u
    return np.array([x[0] + v * np.cos(x[2]) * dt,
                     x[1] + v * np.sin(x[2]) * dt,
                     x[2] + w * dt])


def rollout(x, u, sim_time=SIM_TIME):   # sim_time : 궤적 예측 시간.
    """u를 sim_time 동안 유지했을 때의 궤적. step2 이후에서 사용."""
    traj = [x]
    steps = int(sim_time / DT)
    for _ in range(steps):
        traj.append(motion(traj[-1], u))    # calc를 통해 다음 step trajectory point 반환.
    return np.array(traj)


# ---------------------------------------------------------------- 평가 함수

def min_clearance(traj):
    """궤적상 모든 점에서 가장 가까운 장애물까지의 거리."""
    # traj[:, None, :2] - OBSTACLES[None] -> (steps, n_obs, 2)
    diff = traj[:, None, :2] - OBSTACLES[None, :, :]
    return np.min(np.linalg.norm(diff, axis=2))     # np.linalg.norm : 벡터의 norm calc하는 func.       #norm value 중 minimum value return.


def score(traj, u, cfg):
    """후보 (v, w)의 점수. 탈락이면 None."""
    end = traj[-1]
    clearance = min_clearance(traj)

    # 충돌 궤적은 점수 계산 없이 탈락 (하드 제약)
    if cfg['collision_reject'] and clearance <= ROBOT_RADIUS + SAFETY_MARGIN:
        return None

    to_goal = GOAL - end[:2]

    if cfg['normalize']:
        # 진척(progress): 이 궤적을 따라가면 목표까지 거리가 얼마나 줄어드는가.
        # 정렬(alignment)만 보면 제자리 회전이 만점이 되어 로봇이 출발하지 않는다.
        # nav2 DWB의 GoalDist critic에 해당하는 항.
        d_start = np.linalg.norm(GOAL - traj[0][:2])    #distance btw start & goal.
        d_end = np.linalg.norm(to_goal)                 #distance btw end & goal.
        span = V_MAX * (SIM_TIME if cfg['rollout'] else DT)
        heading = (d_start - d_end) / span          # 직진 접근이면 ~1.0
        clear_s = np.clip(clearance / CLEARANCE_REF, 0.0, 1.0)          #normalization for clearance.
        vel_s = u[0] / V_MAX
    else:
        # 원본 방식: 목표 방향 벡터를 진행 방향에 정사영 -> 거리에 비례해 커진다
        heading = np.dot(to_goal, [np.cos(end[2]), np.sin(end[2])])
        clear_s = clearance
        vel_s = u[0]

    w_h, w_c, w_v = WEIGHTS
    return w_h * heading + w_c * clear_s + w_v * vel_s


# ---------------------------------------------------------------- 속도 윈도우

def velocity_window(v_cur, w_cur, cfg):
    """후보로 삼을 (v, w) 범위. step1에서 dynamic window가 들어간다."""
    if cfg['accel_limit']:
        # 한 제어주기 동안 가속도로 도달 가능한 범위로만 제한
        v_lo = max(0.0, v_cur - A_MAX * DT)
        v_hi = min(V_MAX, v_cur + A_MAX * DT)
        w_lo = max(-W_MAX, w_cur - ALPHA_MAX * DT)
        w_hi = min(W_MAX, w_cur + ALPHA_MAX * DT)
    else:
        # 원본: 매 스텝 전 범위를 훑는다 -> 0에서 1.0 m/s로 점프 가능
        v_lo, v_hi = 0.0, V_MAX
        w_lo, w_hi = -W_MAX, W_MAX
    return np.linspace(v_lo, v_hi, N_V), np.linspace(w_lo, w_hi, N_W)


# ---------------------------------------------------------------- 시뮬레이션

def simulate(cfg):
    x = START.copy()
    v_cur, w_cur = 0.0, 0.0

    path = [x[:2].copy()]
    cmds = []
    best_trajs = []          # 각 스텝에서 선택된 예측 궤적 (시각화용)
    rejected = 0

    for _ in range(int(T_MAX / DT)):
        if np.linalg.norm(GOAL - x[:2]) < GOAL_TOL:
            break

        vs, ws = velocity_window(v_cur, w_cur, cfg)
        best_score, best_u, best_traj = -np.inf, None, None

        for v in vs:
            for w in ws:
                u = (v, w)
                traj = rollout(x, u) if cfg['rollout'] else np.array([x, motion(x, u)])
                s = score(traj, u, cfg)
                if s is None:
                    rejected += 1
                    continue
                if s > best_score:
                    best_score, best_u, best_traj = s, u, traj

        if best_u is None:
            # 모든 후보가 충돌 -> 제동 (가속도 한계 내에서 최대 감속)
            best_u = (max(0.0, v_cur - A_MAX * DT), 0.0)
            best_traj = rollout(x, best_u) if cfg['rollout'] else np.array([x, motion(x, best_u)])

        v_cur, w_cur = best_u
        x = motion(x, best_u)
        path.append(x[:2].copy())
        cmds.append(best_u)
        best_trajs.append(best_traj)

    path = np.array(path)
    cmds = np.array(cmds) if cmds else np.zeros((0, 2))

    # 실제 주행 경로 기준 지표
    traveled_clearance = min_clearance(
        np.hstack([path, np.zeros((len(path), 1))]))
    # 정지(0)에서 출발하므로 첫 명령까지의 변화도 가속으로 센다.
    v_series = np.concatenate([[0.0], cmds[:, 0]]) if len(cmds) else np.zeros(1)
    jerk = (np.max(np.abs(np.diff(v_series))) / DT) if len(v_series) > 1 else 0.0

    return {
        'path': path,
        'trajs': best_trajs,
        'reached': bool(np.linalg.norm(GOAL - path[-1]) < GOAL_TOL),
        'steps': len(path) - 1,
        'length': float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))),
        'clearance': float(traveled_clearance),
        'max_accel': float(jerk),          # 명령된 최대 가속도 [m/s^2]
        'rejected': rejected,
    }


# ---------------------------------------------------------------- 단계 정의

STEPS = {
    'step0_baseline': dict(
        accel_limit=False, rollout=False, collision_reject=False, normalize=False,
        note='original: full-range sampling, 1-step'),
    'step1_accel': dict(
        accel_limit=True, rollout=False, collision_reject=False, normalize=False,
        note='+ accel limits (dynamic window)'),
    'step2_rollout': dict(
        accel_limit=True, rollout=True, collision_reject=False, normalize=False,
        note=f'+ {SIM_TIME}s rollout -> exposes score flaw'),
    'step3_progress': dict(
        accel_limit=True, rollout=True, collision_reject=False, normalize=True,
        note='+ normalized progress score -> fixed'),
    'step4_collision': dict(
        accel_limit=True, rollout=True, collision_reject=True, normalize=True,
        note='+ reject colliding trajectories'),
}


def plot_one(ax, name, cfg, res):
    # 선택된 예측 궤적을 옅게 겹쳐 그려 "무엇을 보고 결정했는지" 보이게 한다
    for traj in res['trajs'][::5]:
        ax.plot(traj[:, 0], traj[:, 1], color='0.8', lw=0.8, zorder=1)

    ax.plot(res['path'][:, 0], res['path'][:, 1], 'b-', lw=2, label='Path', zorder=3)
    ax.plot(*GOAL, 'go', ms=10, label='Goal', zorder=4)
    ax.scatter(OBSTACLES[:, 0], OBSTACLES[:, 1], c='r', s=40, label='Obstacles', zorder=4)
    for ox, oy in OBSTACLES:
        ax.add_patch(plt.Circle((ox, oy), ROBOT_RADIUS, color='r',
                                alpha=0.15, zorder=2))

    hit = 'HIT' if res['clearance'] <= ROBOT_RADIUS else 'ok'
    ax.set_title(
        f"{name}\n{cfg['note']}\n"
        f"goal={'reached' if res['reached'] else 'FAILED'}  "
        f"clear={res['clearance']:.2f}m ({hit})  "
        f"peak_acc={res['max_accel']:.1f}m/s2",
        fontsize=9)
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-0.5, 6)
    ax.set_ylim(-0.5, 6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--show', action='store_true', help='창으로 띄우기')
    ap.add_argument('--only', help='한 단계만 실행')
    args = ap.parse_args()

    if not args.show:
        matplotlib.use('Agg')

    table = dict(STEPS)
    names = [args.only] if args.only else list(table)
    results = {}

    print(f"{'step':<20} {'goal':<7} {'steps':>6} {'len[m]':>7} "
          f"{'clear[m]':>9} {'acc[m/s2]':>10} {'rejected':>9}")
    print('-' * 76)
    for name in names:
        cfg = table[name]
        res = simulate(cfg)
        results[name] = res
        print(f"{name:<20} {'reached' if res['reached'] else 'FAILED':<7} "
              f"{res['steps']:>6} {res['length']:>7.2f} "
              f"{res['clearance']:>9.2f} {res['max_accel']:>10.1f} "
              f"{res['rejected']:>9}")

    ncol = min(3, len(names))
    nrow = (len(names) + ncol - 1) // ncol
    fig, axes = plt.subplots(nrow, ncol, figsize=(5 * ncol, 5 * nrow))
    axes = np.atleast_1d(axes).ravel()

    for ax, name in zip(axes, names):
        plot_one(ax, name, table[name], results[name])
    for ax in axes[len(names):]:
        ax.axis('off')

    axes[0].legend(loc='lower right', fontsize=8)
    fig.suptitle('DWA step-by-step (grey: predicted trajectory chosen at each step)')
    fig.tight_layout()

    out = 'dwa_steps.png'
    fig.savefig(out, dpi=110)
    print(f'\n그림 저장: {out}')
    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
