// Gazebo 없이 planner/controller 단계 조합을 미리 돌려 보는 2D 운동학 시뮬레이터.
//
// 플러그인과 똑같은 코드(line_builder.hpp, follower_core.hpp)를 쓰고, 그 바깥에서
// Nav2 Humble 의 동작을 흉내 낸다.
//   - controller_server : 20Hz 제어 루프, failure_tolerance, 경로 교체 시 goal checker reset
//   - SimpleGoalChecker : xy 0.25m / yaw 0.25rad, stateful
//   - SimpleProgressChecker : 10초 동안 0.5m 를 못 움직이면 실패
//   - velocity_smoother : 0.26 m/s, 1.0 rad/s, 가속도 2.5 / 3.2 (OPEN_LOOP)
//   - BT : 한 번만 계획(plan once) 또는 1Hz 재계획(기본 BT 와 같은 주기)
// 값은 nav2_bringup/params/nav2_params.yaml (Humble) 기본값이다.
//
// 맵: line_tutorial_bringup 의 'line_room' (6m x 6m 방 + 기둥 1개 + 박스 1개).
//
// 흉내 내지 않는 것: 센서, AMCL 오차, costmap inflation, 로봇 동역학.
// (컨트롤러에는 충돌 회피가 없으므로, 장애물은 '부딪혔는지' 판정과 P3 검사에만 쓴다)

#ifndef LINE_TUTORIAL_SIM__KINEMATIC_SIM_HPP_
#define LINE_TUTORIAL_SIM__KINEMATIC_SIM_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "line_follower_controller/follower_core.hpp"
#include "straight_line_planner/line_builder.hpp"

namespace line_tutorial_sim
{

namespace lf = line_follower_controller;
namespace slp = straight_line_planner;
using Pose2D = lf::Pose2D;

struct Obstacle
{
  double x;
  double y;
  double radius;
};

// line_room 월드. line_tutorial_bringup/worlds/generate_line_room.py 와 같은 값이다.
constexpr double kRoomHalf = 3.0;                       // 방 안쪽 반폭 [m]
constexpr Obstacle kPillar{-0.5, 0.0, 0.15};            // S3 용 기둥
constexpr double kBoxX = -2.2, kBoxY = 2.2, kBoxHalf = 0.2;   // 비대칭 박스

// 로봇 중심이 (x, y) 일 때 몸체(반지름 robot_radius)가 벽/기둥/박스에 닿는가
inline bool lineRoomBlocked(double x, double y, double robot_radius)
{
  if (std::hypot(x - kPillar.x, y - kPillar.y) < kPillar.radius + robot_radius) {
    return true;
  }
  if (std::fabs(x - kBoxX) < kBoxHalf + robot_radius &&
    std::fabs(y - kBoxY) < kBoxHalf + robot_radius)
  {
    return true;
  }
  return std::fabs(x) + robot_radius > kRoomHalf || std::fabs(y) + robot_radius > kRoomHalf;
}

struct Scenario
{
  std::string name;
  std::string description;
  Pose2D start;
  Pose2D goal;
};

struct SimConfig
{
  int planner_step{slp::ORIENTED};
  int controller_step{lf::LIMITS};
  bool replan{false};                  // false: 한 번만 계획, true: 1Hz 재계획
  double replan_period{1.0};
  double dt{0.05};                     // controller_frequency 20Hz
  double t_max{60.0};
  double interpolation_resolution{0.1};
  double local_extent{1.5};            // local costmap 3m x 3m 의 절반
  double xy_goal_tolerance{0.25};
  double yaw_goal_tolerance{0.25};
  double progress_radius{0.5};
  double progress_time{10.0};
  double failure_tolerance{0.3};
  double robot_radius{0.22};
  double smoother_max_v{0.26};
  double smoother_max_w{1.0};
  double smoother_accel_v{2.5};
  double smoother_accel_w{3.2};
  lf::Params controller;               // step 은 controller_step 으로 덮어쓴다
};

struct Sample
{
  double t;
  Pose2D pose;
  double v;
  double w;
  std::string state;
};

struct SimResult
{
  // reached / plan_rejected / controller_failed / no_progress / collision / timeout
  std::string outcome;
  std::string detail;
  double time{0.0};
  double max_cte{0.0};          // 처음 그은 직선(출발점 -> goal)에서 벗어난 최대 거리 [m]
  double rms_cte{0.0};
  double max_cte_latest{0.0};   // 가장 최근에 받은 경로에서 벗어난 최대 거리 [m]
  double final_xy_err{0.0};
  double final_yaw_err{0.0};
  double peak_cmd_accel{0.0};   // 컨트롤러가 낸 선속도 명령의 최대 증가율 [m/s^2]
  double peak_cmd_decel{0.0};   // 최대 감소율 [m/s^2]
  std::vector<Sample> trace;

  bool reached() const {return outcome == "reached";}
};

inline double distToSegment(const Pose2D & p, const Pose2D & a, const Pose2D & b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double len2 = vx * vx + vy * vy;
  double t = len2 > 0.0 ? ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(p.x - (a.x + t * vx), p.y - (a.y + t * vy));
}

// planner_server 역할: 현재 로봇 pose 에서 goal 까지 직선을 만든다.
inline bool planLine(
  const SimConfig & cfg, const Scenario & sc, const Pose2D & from,
  std::vector<Pose2D> & out, std::string & why)
{
  const slp::Pose2D s{from.x, from.y, from.yaw};
  const slp::Pose2D g{sc.goal.x, sc.goal.y, sc.goal.yaw};
  if (cfg.planner_step >= slp::COLLISION) {
    // costmap 의 INSCRIBED 이상 = 장애물 표면에서 robot_radius 이내
    auto blocked = [&](double x, double y) {return lineRoomBlocked(x, y, cfg.robot_radius);};
    if (slp::firstBlockedDistance(s, g, 0.025, blocked) >= 0.0) {
      why = "straight line is blocked";
      return false;
    }
  }
  out.clear();
  for (const auto & p : slp::buildLine(s, g, cfg.interpolation_resolution, cfg.planner_step)) {
    out.push_back({p.x, p.y, p.yaw});
  }
  return true;
}

inline SimResult simulate(const Scenario & sc, const SimConfig & cfg)
{
  SimResult r;
  lf::Params params = cfg.controller;
  params.step = cfg.controller_step;

  Pose2D robot = sc.start;
  lf::Velocity vel;              // 실제 로봇 속도 (velocity smoother 통과 후)
  double last_cmd_v = 0.0;
  std::vector<Pose2D> plan;
  std::string why;
  double cte_sq_sum = 0.0;
  int cte_n = 0;

  auto finish = [&](const std::string & outcome, const std::string & detail, double t) {
      r.outcome = outcome;
      r.detail = detail;
      r.time = t;
      r.final_xy_err = std::hypot(robot.x - sc.goal.x, robot.y - sc.goal.y);
      r.final_yaw_err = std::fabs(lf::normalizeAngle(robot.yaw - sc.goal.yaw));
      r.rms_cte = cte_n > 0 ? std::sqrt(cte_sq_sum / cte_n) : 0.0;
      return r;
    };

  if (!planLine(cfg, sc, robot, plan, why)) {
    return finish("plan_rejected", why, 0.0);
  }
  Pose2D plan_start = robot;     // 최근 경로의 시작점 (CTE 비교용)
  double last_plan_t = 0.0;
  bool check_xy = true;          // SimpleGoalChecker 의 stateful 플래그
  Pose2D baseline = robot;       // SimpleProgressChecker 의 기준점
  double baseline_t = 0.0;
  double last_valid_cmd_t = 0.0;

  const int steps = static_cast<int>(cfg.t_max / cfg.dt);
  for (int k = 0; k < steps; ++k) {
    const double t = k * cfg.dt;

    // BT: 1Hz 재계획. 새 경로가 들어오면 controller_server 가 goal checker 를 reset 한다.
    if (cfg.replan && k > 0 && t - last_plan_t >= cfg.replan_period - 1e-9) {
      if (!planLine(cfg, sc, robot, plan, why)) {
        return finish("plan_rejected", why, t);
      }
      plan_start = robot;
      last_plan_t = t;
      check_xy = true;
    }

    // progress checker
    if (std::hypot(robot.x - baseline.x, robot.y - baseline.y) > cfg.progress_radius) {
      baseline = robot;
      baseline_t = t;
    } else if (t - baseline_t > cfg.progress_time) {
      return finish("no_progress", "moved less than 0.5 m in 10 s", t);
    }

    // controller 플러그인과 같은 순서: pruning -> local window -> 제어 법칙
    const size_t closest = lf::closestIndex(plan, robot);
    plan.erase(plan.begin(), plan.begin() + static_cast<long>(closest));
    const auto local = lf::localWindow(plan, 0, robot, cfg.local_extent);
    const Pose2D goal_local = lf::toFrame(robot, plan.back());
    const auto cmd = lf::computeCommand(
      params, local, goal_local, vel, cfg.dt, cfg.xy_goal_tolerance);

    double v_cmd = 0.0;
    double w_cmd = 0.0;
    if (cmd.ok) {
      v_cmd = cmd.v;
      w_cmd = cmd.w;
      last_valid_cmd_t = t;
    } else if (t - last_valid_cmd_t > cfg.failure_tolerance) {
      return finish("controller_failed", cmd.state, t);
    }
    r.peak_cmd_accel = std::max(r.peak_cmd_accel, (v_cmd - last_cmd_v) / cfg.dt);
    r.peak_cmd_decel = std::max(r.peak_cmd_decel, (last_cmd_v - v_cmd) / cfg.dt);
    last_cmd_v = v_cmd;
    r.trace.push_back({t, robot, v_cmd, w_cmd, cmd.state});

    // goal checker (같은 pose 로 판단)
    {
      const Pose2D & end = plan.back();
      bool reached = true;
      if (check_xy) {
        if (std::hypot(robot.x - end.x, robot.y - end.y) > cfg.xy_goal_tolerance) {
          reached = false;
        } else {
          check_xy = false;
        }
      }
      if (reached &&
        std::fabs(lf::normalizeAngle(robot.yaw - end.yaw)) <= cfg.yaw_goal_tolerance)
      {
        return finish("reached", "", t);
      }
    }

    // velocity smoother
    double v = std::clamp(v_cmd, -cfg.smoother_max_v, cfg.smoother_max_v);
    double w = std::clamp(w_cmd, -cfg.smoother_max_w, cfg.smoother_max_w);
    v = std::clamp(v, vel.v - cfg.smoother_accel_v * cfg.dt, vel.v + cfg.smoother_accel_v * cfg.dt);
    w = std::clamp(w, vel.w - cfg.smoother_accel_w * cfg.dt, vel.w + cfg.smoother_accel_w * cfg.dt);
    vel = {v, w};

    // 차동구동 적분 (중간 각도 사용)
    const double mid_yaw = robot.yaw + 0.5 * w * cfg.dt;
    robot.x += v * std::cos(mid_yaw) * cfg.dt;
    robot.y += v * std::sin(mid_yaw) * cfg.dt;
    robot.yaw = lf::normalizeAngle(robot.yaw + w * cfg.dt);

    if (lineRoomBlocked(robot.x, robot.y, cfg.robot_radius)) {
      return finish("collision", "hit the pillar, the box or a wall", t);
    }

    const double cte = distToSegment(robot, sc.start, sc.goal);
    r.max_cte = std::max(r.max_cte, cte);
    cte_sq_sum += cte * cte;
    ++cte_n;
    r.max_cte_latest = std::max(r.max_cte_latest, distToSegment(robot, plan_start, sc.goal));
  }
  return finish("timeout", "", cfg.t_max);
}

// 튜토리얼에서 쓰는 시나리오 (line_room 의 map frame 좌표).
// 출발점 A = 스폰 위치 (-2.0, -1.0, 0도), B = A 에서 3m 앞 (1.0, -1.0, 0도).
// Gazebo 에서는 send_scenario.py 가 로봇을 출발점까지 옮긴 뒤 같은 goal 을 보낸다.
inline std::vector<Scenario> defaultScenarios()
{
  const double deg = M_PI / 180.0;
  const Pose2D A{-2.0, -1.0, 0.0};
  const Pose2D B{1.0, -1.0, 0.0};
  return {
    {"S1_straight", "A -> B: 3 m straight ahead along map +x", A, {1.0, -1.0, 0.0}},
    {"S2_behind", "B -> A: goal straight behind the robot", B, {-2.0, -1.0, 180 * deg}},
    {"S3_pillar", "A -> (1, 1): the pillar sits in the middle of the line", A,
      {1.0, 1.0, std::atan2(2.0, 3.0)}},
    {"S4_goal_yaw", "A -> B, but the goal yaw is +90 deg", A, {1.0, -1.0, 90 * deg}},
    {"S5_offset", "B -> (1.3, 2): the line is 84 deg to the left of the robot", B,
      {1.3, 2.0, std::atan2(3.0, 0.3)}},
  };
}

}  // namespace line_tutorial_sim

#endif  // LINE_TUTORIAL_SIM__KINEMATIC_SIM_HPP_
