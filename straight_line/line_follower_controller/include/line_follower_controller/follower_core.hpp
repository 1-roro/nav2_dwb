// 경로 추종 제어 법칙 (ROS 의존성 없음).
//
// 플러그인(line_follower_controller.cpp)과 오프라인 시뮬레이터(line_tutorial_sim)가
// 같은 코드를 쓰도록 헤더 하나에 모아 두었다.
//
// 단계 (파라미터 `<controller>.step` 으로 선택)
//   0 GOAL_SEEK      경로를 무시하고 최종 goal 로 P 제어            <- baseline
//   1 HEADING_TRACK  가장 가까운 path pose 의 orientation + 횡오차  <- P1 경로(yaw=0)와 만나면 실패
//   2 PURE_PURSUIT   lookahead 점을 향한 곡률 k = 2y / L^2          <- goal 이 등 뒤 / goal yaw 문제
//   3 ROTATE         2 + 출발 시 제자리 회전, goal 에서 yaw 맞추기   <- 2 의 결함을 고친다
//   4 LIMITS         3 + 가속도 제한, goal 접근 감속                 <- 사실상 mini RPP
//
// 좌표계: 아래 함수들은 '로봇 좌표계'(로봇이 원점, +x 가 정면)로 표현된 경로를 받는다.

#ifndef LINE_FOLLOWER_CONTROLLER__FOLLOWER_CORE_HPP_
#define LINE_FOLLOWER_CONTROLLER__FOLLOWER_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace line_follower_controller
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

enum Step : int
{
  GOAL_SEEK = 0,
  HEADING_TRACK = 1,
  PURE_PURSUIT = 2,
  ROTATE = 3,
  LIMITS = 4,
};

inline const char * stepName(int step)
{
  switch (step) {
    case GOAL_SEEK: return "C0_goal_seek";
    case HEADING_TRACK: return "C1_heading_track";
    case PURE_PURSUIT: return "C2_pure_pursuit";
    case ROTATE: return "C3_rotate";
    default: return "C4_limits";
  }
}

inline double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// ref 기준 좌표계로 p 를 표현한다 (= ref^-1 * p). ref 가 로봇 pose 이면 로봇 좌표계가 된다.
inline Pose2D toFrame(const Pose2D & ref, const Pose2D & p)
{
  const double dx = p.x - ref.x;
  const double dy = p.y - ref.y;
  const double c = std::cos(ref.yaw);
  const double s = std::sin(ref.yaw);
  return {c * dx + s * dy, -s * dx + c * dy, normalizeAngle(p.yaw - ref.yaw)};
}

// ---------------------------------------------------------------- 경로 전처리
// RPP / DWB 의 transformGlobalPlan 과 같은 방식.

// 경로에서 로봇에 가장 가까운 pose 의 index.
// 경로 전체를 훑는 단순한 버전이다. 직선 경로에서는 문제가 없지만, 경로가 되돌아오는 모양이면
// 나중 구간을 잡을 수 있다. RPP 는 max_robot_pose_search_dist 로 탐색 범위를 제한한다.
inline size_t closestIndex(const std::vector<Pose2D> & plan, const Pose2D & robot)
{
  size_t best = 0;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < plan.size(); ++i) {
    const double d = std::hypot(plan[i].x - robot.x, plan[i].y - robot.y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

// plan[begin] 부터 로봇에서 max_dist 이내인 '연속 구간'만 로봇 좌표계로 바꿔 반환한다.
// max_dist 는 local costmap 크기의 절반이다. 기본 설정(3m x 3m)이면 1.5m.
// 이 범위 밖의 pose 를 처음 만나면 거기서 멈춘다. 그래서 두 점짜리 경로(P0)는 goal 이 잘려 나간다.
inline std::vector<Pose2D> localWindow(
  const std::vector<Pose2D> & plan, size_t begin, const Pose2D & robot, double max_dist)
{
  std::vector<Pose2D> local;
  for (size_t i = begin; i < plan.size(); ++i) {
    if (std::hypot(plan[i].x - robot.x, plan[i].y - robot.y) > max_dist) {
      break;
    }
    local.push_back(toFrame(robot, plan[i]));
  }
  return local;
}

// ---------------------------------------------------------------- 제어 법칙

struct Params
{
  int step{LIMITS};
  double desired_linear_vel{0.25};          // [m/s]
  double max_angular_vel{1.0};              // [rad/s]
  // C0, C1
  double k_heading{1.5};                    // 방향 오차 이득
  double k_cross_track{2.0};                // 횡오차 이득 (C1)
  // C2+
  double lookahead_dist{0.6};               // [m]
  // C3+
  double rotate_to_heading_min_angle{0.785};  // 이보다 크게 틀어져 있으면 제자리 회전 [rad]
  double rotate_to_heading_angular_vel{1.0};  // 제자리 회전 속도 [rad/s]
  // C4
  double max_linear_accel{0.5};             // [m/s^2]
  double max_angular_accel{3.2};            // [rad/s^2]
  double approach_dist{0.6};                // goal 이 이 거리 안이면 감속 [m]
  double min_approach_linear_vel{0.05};     // 감속해도 이 속도 밑으로는 내리지 않음 [m/s]
};

struct Velocity
{
  double v{0.0};
  double w{0.0};
};

struct Command
{
  double v{0.0};
  double w{0.0};
  bool ok{true};               // false 이면 플러그인이 예외를 던진다
  const char * state{""};      // 현재 어떤 분기로 계산했는지 (로그/디버깅용)
  bool has_carrot{false};
  Pose2D carrot;               // C2+ 의 lookahead 점 (로봇 좌표계)
};

namespace detail
{
inline double clampAbs(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

inline double sign(double value)
{
  return value >= 0.0 ? 1.0 : -1.0;
}
}  // namespace detail

// local         : localWindow() 결과 (로봇 좌표계)
// goal          : 전역 경로의 마지막 pose (로봇 좌표계)
// current       : 현재 속도 (C4 가속도 제한에 사용)
// dt            : 제어 주기 [s]
// goal_dist_tol : goal checker 의 xy 허용오차 (C3 에서 'goal 에 도착함' 판단에 사용)
inline Command computeCommand(
  const Params & p, const std::vector<Pose2D> & local, const Pose2D & goal,
  const Velocity & current, double dt, double goal_dist_tol)
{
  using detail::clampAbs;
  using detail::sign;
  Command cmd;
  const double goal_dist = std::hypot(goal.x, goal.y);

  // ---- C0: 경로를 무시하고 최종 goal 을 향해 간다
  if (p.step <= GOAL_SEEK) {
    const double angle = std::atan2(goal.y, goal.x);
    cmd.v = p.desired_linear_vel * std::max(0.0, std::cos(angle));
    cmd.w = clampAbs(p.k_heading * angle, p.max_angular_vel);
    cmd.state = "goal_seek";
    return cmd;
  }

  if (local.empty()) {
    // RPP 의 "Resulting plan has 0 poses in it." 와 같은 상황
    cmd.ok = false;
    cmd.state = "no path poses inside the local costmap";
    return cmd;
  }

  // ---- C1: 가장 가까운 pose 의 orientation 을 따라가며 횡오차를 줄인다
  if (p.step == HEADING_TRACK) {
    const Pose2D & ref = local.front();      // pruning 후 첫 pose = 가장 가까운 pose
    const double heading_err = ref.yaw;      // 로봇 좌표계에서 본 경로 방향 = 방향 오차
    // 경로 직선이 로봇의 왼쪽(+)/오른쪽(-)으로 얼마나 떨어져 있는가 (경로 법선 방향 거리)
    const double lateral = ref.y * std::cos(ref.yaw) - ref.x * std::sin(ref.yaw);
    cmd.v = p.desired_linear_vel;
    cmd.w = clampAbs(p.k_heading * heading_err + p.k_cross_track * lateral, p.max_angular_vel);
    cmd.state = "heading_track";
    return cmd;
  }

  // ---- C2+: pure pursuit
  // carrot = lookahead_dist 이상 떨어진 첫 pose. 없으면 마지막 pose. (RPP getLookAheadPoint 와 동일)
  auto it = std::find_if(
    local.begin(), local.end(), [&](const Pose2D & q) {
      return std::hypot(q.x, q.y) >= p.lookahead_dist;
    });
  cmd.carrot = (it != local.end()) ? *it : local.back();
  cmd.has_carrot = true;
  const double carrot_angle = std::atan2(cmd.carrot.y, cmd.carrot.x);
  const double L2 = cmd.carrot.x * cmd.carrot.x + cmd.carrot.y * cmd.carrot.y;

  // C4: 각가속도 제한 (제자리 회전 분기에서 사용)
  auto limit_w = [&](double w) {
      if (p.step >= LIMITS) {
        w = std::clamp(w, current.w - p.max_angular_accel * dt, current.w + p.max_angular_accel * dt);
      }
      return w;
    };

  if (p.step >= ROTATE && goal_dist <= goal_dist_tol) {
    // C3: goal 의 xy 에는 도착했다 -> 그 자리에서 goal yaw 로 회전
    cmd.v = 0.0;
    cmd.w = limit_w(sign(goal.yaw) * p.rotate_to_heading_angular_vel);
    cmd.state = "rotate_to_goal";
  } else if (p.step >= ROTATE && std::fabs(carrot_angle) > p.rotate_to_heading_min_angle) {
    // C3: 경로 쪽으로 크게 틀어져 있다 -> 먼저 제자리에서 돌린다
    cmd.v = 0.0;
    cmd.w = limit_w(sign(carrot_angle) * p.rotate_to_heading_angular_vel);
    cmd.state = "rotate_to_path";
  } else if (L2 < 1e-6) {
    // carrot 이 로봇 위에 있다 = 앞으로 따라갈 점이 없다 (경로가 너무 성긴 경우)
    cmd.v = 0.0;
    cmd.w = 0.0;
    cmd.state = "carrot_at_robot";
  } else {
    const double curvature = 2.0 * cmd.carrot.y / L2;
    double v = p.desired_linear_vel;
    if (p.step >= LIMITS) {
      // C4: goal 에 가까워지면 감속
      if (goal_dist < p.approach_dist) {
        v = std::max(std::min(v, p.min_approach_linear_vel), v * goal_dist / p.approach_dist);
      }
      // C4: 선가속도 제한
      v = std::clamp(v, current.v - p.max_linear_accel * dt, current.v + p.max_linear_accel * dt);
      v = std::max(v, 0.0);
    }
    double w = v * curvature;
    // 각속도 상한에 걸리면 v 도 같은 비율로 줄여서 곡률(= 지나가는 원호)을 유지한다
    if (std::fabs(w) > p.max_angular_vel) {
      v *= p.max_angular_vel / std::fabs(w);
      w = sign(w) * p.max_angular_vel;
    }
    cmd.v = v;
    cmd.w = w;
    cmd.state = "pure_pursuit";
  }
  return cmd;
}

}  // namespace line_follower_controller

#endif  // LINE_FOLLOWER_CONTROLLER__FOLLOWER_CORE_HPP_
