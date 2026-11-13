#ifndef BT_MISSION__DOCK_CONTROL_HPP_
#define BT_MISSION__DOCK_CONTROL_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

// 도킹에 같이 쓰는 계산 (ROS 와 무관한 계산만. dock_nodes.cpp (BackOff) 와 aruco_dock_control.hpp 가 쓰고,
// test_dock_control.cpp 가 시험한다)
//
// 메카넘(홀로노믹) 로봇은 앞뒤(vx), 옆(vy), 회전(wz)을 한꺼번에 낼 수 있다. 그래서 차동 구동처럼
// "돌고 → 가고 → 돌기" 를 하지 않고, 목표 자세까지의 오차를 세 축에서 동시에 줄인다.
//
//   ex, ey : 목표 위치가 로봇 좌표계에서 어디 있나 (앞 +, 왼쪽 +)
//   eyaw   : 목표 방향 - 지금 방향
//   (vx, vy) = k_lin (ex, ey)  크기가 max_lin 을 넘으면 방향은 그대로 두고 크기만 줄인다
//   wz       = k_ang eyaw      max_ang 까지
//
// P 제어라 목표에 가까울수록 느려지며 부드럽게 선다. 다만 아주 느린 명령은 실제 모터가 따라오지 못하므로
// (마찰, 데드밴드) 허용오차 밖에서는 min_lin, min_ang 아래로 줄이지 않는다.
// 가속도는 축마다 직전 명령에서 lin_accel, ang_accel 로 제한한다.

namespace bt_mission
{
namespace dock
{

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Cmd
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct ControlParams
{
  double k_lin{1.0};           // [1/s]
  double k_ang{1.5};           // [1/s]
  double max_lin{0.15};        // vx, vy 를 합친 크기의 상한 [m/s]
  double max_ang{0.5};         // [rad/s]
  double min_lin{0.01};        // 허용오차 밖에서 이보다 느리게 하지 않는다 [m/s]
  double min_ang{0.03};        // [rad/s]
  double lin_accel{0.3};       // [m/s^2]
  double ang_accel{1.0};       // [rad/s^2]
  double xy_tolerance{0.01};   // [m]
  double yaw_tolerance{0.02};  // [rad]
};

inline double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// 목표 자세의 오차를 로봇 좌표계로 (x 앞, y 왼쪽)
inline Pose2 errorInRobotFrame(const Pose2 & robot, const Pose2 & target)
{
  const double dx = target.x - robot.x;
  const double dy = target.y - robot.y;
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);
  return {c * dx + s * dy, -s * dx + c * dy, normalizeAngle(target.yaw - robot.yaw)};
}

inline bool withinTolerance(const Pose2 & error, const ControlParams & p)
{
  return std::hypot(error.x, error.y) <= p.xy_tolerance && std::fabs(error.yaw) <= p.yaw_tolerance;
}

// 한 주기(dt) 의 명령
inline Cmd holonomicStep(const Pose2 & error, const Cmd & last, const ControlParams & p, double dt)
{
  Cmd want;
  const double dist = std::hypot(error.x, error.y);
  if (dist > p.xy_tolerance) {
    const double speed = std::clamp(p.k_lin * dist, p.min_lin, p.max_lin);
    want.vx = speed * error.x / dist;
    want.vy = speed * error.y / dist;
  }
  if (std::fabs(error.yaw) > p.yaw_tolerance) {
    want.wz = std::copysign(std::clamp(p.k_ang * std::fabs(error.yaw), p.min_ang, p.max_ang), error.yaw);
  }
  auto limit = [dt](double target, double previous, double accel) {
      return std::clamp(target, previous - accel * dt, previous + accel * dt);
    };
  return {limit(want.vx, last.vx, p.lin_accel), limit(want.vy, last.vy, p.lin_accel),
    limit(want.wz, last.wz, p.ang_accel)};
}

inline double median(std::vector<double> v)
{
  std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size() / 2), v.end());
  return v[v.size() / 2];
}

// 로봇 좌표계의 점 중에서, 뒤로 distance 만큼 갈 때 몸이 쓸고 가는 사각형 안에 있는 것의 수.
// 사각형: 로봇 뒤 끝(x = -rear)에서 distance + clearance 더 뒤까지, 폭 ±(half_width + clearance).
// 로봇 몸 안(x > -rear)의 점은 세지 않는다 (레이저가 제 몸을 보는 일은 없다)
struct Point2
{
  double x{0.0};
  double y{0.0};
};

inline int pointsBehind(
  const std::vector<Point2> & points, double rear, double half_width, double distance, double clearance)
{
  int count = 0;
  for (const auto & p : points) {
    if (p.x <= -rear && p.x >= -(rear + distance + clearance) && std::fabs(p.y) <= half_width + clearance) {
      ++count;
    }
  }
  return count;
}

}  // namespace dock
}  // namespace bt_mission

#endif  // BT_MISSION__DOCK_CONTROL_HPP_
