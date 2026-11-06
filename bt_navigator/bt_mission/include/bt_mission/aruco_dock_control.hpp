#ifndef BT_MISSION__ARUCO_DOCK_CONTROL_HPP_
#define BT_MISSION__ARUCO_DOCK_CONTROL_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "bt_mission/dock_control.hpp"

// ArUco 도킹의 계산 (ROS 와 무관. aruco_dock_nodes.cpp 가 쓰고 test_aruco_dock_control.cpp 가 시험한다).
// ARUCO_DOCKING.md 의 2 ~ 4장.
//
// 1. dock 의 자리 추정: 움직이지 않는 물체 하나의 자세(odom 의 x, y, yaw)를 Kalman 필터로 잡는다.
//    측정마다 공분산(aruco_detector.py 가 모서리 잡음에서 계산)이 달라서, 가까이서 본 정확한 측정이 저절로
//    크게 반영된다. odom 이 조금씩 흐르는 것은 random walk 로 둔다. 너무 먼 측정은 Mahalanobis 거리로 버리되,
//    연달아 버리면(처음 값이 틀렸으면) 다시 잡는다.
// 2. 제어: 오차를 dock 좌표계(along = 법선 방향 남은 거리, lat = 옆, yaw)로 보고, 메카넘이라 세 축을 동시에 줄인다.
//    다만 "깔때기": 옆과 방향 오차가 남은 거리에 비례하는 폭 안에 들어야 앞으로(dock 쪽으로) 간다.
//    축마다 최대 속도, 가속도, 최소 속도(데드밴드), 명령 배율(실제 로봇에서 잰 미끄러짐 보정)이 다르다.

namespace bt_mission
{
namespace aruco_dock
{

using dock::Cmd;
using dock::normalizeAngle;
using dock::Pose2;

using Mat3 = std::array<double, 9>;      // 3x3, 행 우선. 순서 (x, y, yaw)

inline Mat3 diag3(double a, double b, double c) {return {a, 0, 0, 0, b, 0, 0, 0, c};}

inline Mat3 add(const Mat3 & a, const Mat3 & b)
{
  Mat3 out;
  for (int i = 0; i < 9; ++i) {
    out[i] = a[i] + b[i];
  }
  return out;
}

inline Mat3 mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      for (int k = 0; k < 3; ++k) {
        out[r * 3 + c] += a[r * 3 + k] * b[k * 3 + c];
      }
    }
  }
  return out;
}

inline std::optional<Mat3> inverse3(const Mat3 & m)
{
  const double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], i = m[8];
  const double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (std::fabs(det) < 1e-30) {
    return std::nullopt;
  }
  return Mat3{(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det,
    (f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det,
    (d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det};
}

inline std::array<double, 3> mulVec(const Mat3 & m, const std::array<double, 3> & v)
{
  return {m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
}

// 2D 자세: a 다음에 b (b 는 a 좌표계에서)
inline Pose2 compose(const Pose2 & a, const Pose2 & b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalizeAngle(a.yaw + b.yaw)};
}

inline Pose2 inverse(const Pose2 & a)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {-(c * a.x + s * a.y), -(-s * a.x + c * a.y), normalizeAngle(-a.yaw)};
}

// p 를 frame 좌표계에서 본 것 (예: 로봇을 dock 좌표계에서)
inline Pose2 relative(const Pose2 & frame, const Pose2 & p) {return compose(inverse(frame), p);}

// 측정 하나: dock 좌표계 D 의 원점과 방향 (odom), 그 공분산
struct Measurement
{
  Pose2 pose;
  Mat3 cov{};
};

inline double mahalanobis2(const Pose2 & mean, const Mat3 & cov, const Pose2 & z)
{
  const auto inv = inverse3(cov);
  if (!inv) {
    return 1e9;
  }
  const std::array<double, 3> y{z.x - mean.x, z.y - mean.y, normalizeAngle(z.yaw - mean.yaw)};
  const auto t = mulVec(*inv, y);
  return y[0] * t[0] + y[1] * t[1] + y[2] * t[2];
}

// 움직이지 않는 dock 의 자세 (odom). 상태 3개 (x, y, yaw), 관측 = 상태
class DockFilter
{
public:
  bool initialized() const {return initialized_;}
  const Pose2 & mean() const {return mean_;}
  const Mat3 & cov() const {return cov_;}
  int rejectedInARow() const {return rejected_in_a_row_;}

  void reset(const Pose2 & z, const Mat3 & cov)
  {
    mean_ = z;
    cov_ = cov;
    initialized_ = true;
    rejected_in_a_row_ = 0;
  }

  // dt 초 동안 odom 이 흐를 수 있는 만큼 불확실도를 키운다 (q: 1초에 [m], [rad] 의 표준편차, random walk)
  void predict(double dt, double q_xy, double q_yaw)
  {
    if (dt <= 0.0) {
      return;
    }
    cov_ = add(cov_, diag3(q_xy * q_xy * dt, q_xy * q_xy * dt, q_yaw * q_yaw * dt));
  }

  enum class Result {Accepted, Rejected, Reinitialized};

  // gate: Mahalanobis 거리 제곱의 문턱 (자유도 3, 99.9 % 면 16.3). reinit_after 번 연달아 버리면 그 측정으로 다시 잡는다
  Result update(const Pose2 & z, const Mat3 & r, double gate, int reinit_after)
  {
    if (!initialized_) {
      reset(z, r);
      return Result::Reinitialized;
    }
    const Mat3 s = add(cov_, r);
    if (mahalanobis2(mean_, s, z) > gate) {
      if (++rejected_in_a_row_ >= reinit_after) {
        reset(z, r);
        return Result::Reinitialized;
      }
      return Result::Rejected;
    }
    rejected_in_a_row_ = 0;
    const auto s_inv = inverse3(s);
    if (!s_inv) {
      return Result::Rejected;
    }
    const Mat3 k = mul(cov_, *s_inv);
    const std::array<double, 3> y{z.x - mean_.x, z.y - mean_.y, normalizeAngle(z.yaw - mean_.yaw)};
    const auto dx = mulVec(k, y);
    mean_ = {mean_.x + dx[0], mean_.y + dx[1], normalizeAngle(mean_.yaw + dx[2])};
    Mat3 i_minus_k = diag3(1.0, 1.0, 1.0);
    for (int i = 0; i < 9; ++i) {
      i_minus_k[i] -= k[i];
    }
    cov_ = mul(i_minus_k, cov_);
    for (int r_ = 0; r_ < 3; ++r_) {          // 대칭으로
      for (int c = r_ + 1; c < 3; ++c) {
        const double v = 0.5 * (cov_[r_ * 3 + c] + cov_[c * 3 + r_]);
        cov_[r_ * 3 + c] = cov_[c * 3 + r_] = v;
      }
    }
    return Result::Accepted;
  }

private:
  bool initialized_{false};
  Pose2 mean_;
  Mat3 cov_{};
  int rejected_in_a_row_{0};
};

// 측정 여러 개를 하나로 (정지해서 본 것). 중앙값에서 Mahalanobis 거리 gate 안의 것만 정보 가중 평균
// (Σ = (Σ Rᵢ⁻¹)⁻¹, x = Σ Σ Rᵢ⁻¹ zᵢ). 쓴 측정 수를 used 로
inline std::optional<Measurement> fuse(const std::vector<Measurement> & ms, double gate, int * used = nullptr)
{
  if (ms.empty()) {
    return std::nullopt;
  }
  std::vector<double> xs, ys, yaws;
  for (const auto & m : ms) {
    xs.push_back(m.pose.x);
    ys.push_back(m.pose.y);
    yaws.push_back(normalizeAngle(m.pose.yaw - ms.front().pose.yaw));
  }
  const Pose2 mid{dock::median(xs), dock::median(ys), normalizeAngle(ms.front().pose.yaw + dock::median(yaws))};
  Mat3 info{};
  std::array<double, 3> weighted{0, 0, 0};
  int n = 0;
  for (const auto & m : ms) {
    if (mahalanobis2(mid, m.cov, m.pose) > gate) {
      continue;
    }
    const auto r_inv = inverse3(m.cov);
    if (!r_inv) {
      continue;
    }
    const std::array<double, 3> z{m.pose.x - mid.x, m.pose.y - mid.y, normalizeAngle(m.pose.yaw - mid.yaw)};
    const auto t = mulVec(*r_inv, z);
    info = add(info, *r_inv);
    for (int i = 0; i < 3; ++i) {
      weighted[i] += t[i];
    }
    ++n;
  }
  if (used) {
    *used = n;
  }
  const auto cov = n > 0 ? inverse3(info) : std::nullopt;
  if (!cov) {
    return std::nullopt;
  }
  const auto d = mulVec(*cov, weighted);
  return Measurement{{mid.x + d[0], mid.y + d[1], normalizeAngle(mid.yaw + d[2])}, *cov};
}

// ---------------------------------------------------------------------------------------------
// 제어

struct MecanumParams
{
  double k_along{0.8};         // [1/s]
  double k_lat{1.2};           // [1/s] 옆을 앞보다 빨리 맞춘다 (깔때기 안으로)
  double k_yaw{1.5};           // [1/s]
  double max_vx{0.15};         // 로봇 앞뒤 [m/s]
  double max_vy{0.10};         // 로봇 옆 [m/s]. 메카넘은 옆으로 갈 때 롤러가 더 미끄러지므로 낮게
  double max_wz{0.4};          // [rad/s]
  double min_vx{0.010};        // 허용오차 밖에서 이보다 느리게 내지 않는다 (모터 데드밴드) [m/s]
  double min_vy{0.015};        // 옆은 롤러 마찰 때문에 더 크게
  double min_wz{0.03};         // [rad/s]
  double accel_x{0.3};         // [m/s^2]
  double accel_y{0.2};
  double accel_wz{1.0};        // [rad/s^2]
  double scale_vx{1.0};        // 명령 배율 = 명령한 속도 / 실제로 나온 속도 (실제 로봇에서 잰 값. 1 이면 보정 없음)
  double scale_vy{1.0};
  double scale_wz{1.0};
  double tol_along{0.003};     // [m]
  double tol_lat{0.002};       // [m] 접점이 옆으로 어긋나면 안 되므로 가장 작게
  double tol_yaw{0.005};       // [rad]
  // 깔때기: 남은 거리(along)가 d 일 때 앞으로 가도 되는 옆 오차 폭 = funnel_lat0 + funnel_lat_slope * d
  double funnel_lat0{0.004};
  double funnel_lat_slope{0.15};
  double funnel_yaw0{0.01};    // [rad]
  double funnel_yaw_slope{0.25};   // [rad/m]
};

// 로봇이 목표에서 얼마나 벗어났나 (dock 좌표계). along > 0: 아직 목표보다 dock 에서 멀다
struct DockErrors
{
  double along{0.0};
  double lat{0.0};
  double yaw{0.0};
};

inline DockErrors dockErrors(const Pose2 & robot_in_dock, const Pose2 & target_in_dock)
{
  return {robot_in_dock.x - target_in_dock.x, robot_in_dock.y - target_in_dock.y,
    normalizeAngle(robot_in_dock.yaw - target_in_dock.yaw)};
}

inline bool withinTolerance(const DockErrors & e, const MecanumParams & p)
{
  return std::fabs(e.along) <= p.tol_along && std::fabs(e.lat) <= p.tol_lat && std::fabs(e.yaw) <= p.tol_yaw;
}

// |e| 가 band 의 절반 안이면 1, band 이상이면 0, 그 사이는 직선
inline double funnelGate(double e, double band)
{
  if (band <= 0.0) {
    return 0.0;
  }
  return std::clamp(2.0 - 2.0 * std::fabs(e) / band, 0.0, 1.0);
}

inline double funnelGate(const DockErrors & e, const MecanumParams & p)
{
  const double d = std::max(e.along, 0.0);
  return funnelGate(e.lat, p.funnel_lat0 + p.funnel_lat_slope * d) *
         funnelGate(e.yaw, p.funnel_yaw0 + p.funnel_yaw_slope * d);
}

// 한 주기의 명령 (로봇 좌표계 vx, vy, wz). robot_yaw_in_dock: dock 좌표계에서 본 로봇 방향
inline Cmd mecanumStep(
  const DockErrors & e, double robot_yaw_in_dock, const Cmd & last, const MecanumParams & p, double dt)
{
  Cmd want;
  if (!withinTolerance(e, p)) {
    // dock 좌표계에서 원하는 속도
    const double gate = funnelGate(e, p);
    double v_along = -p.k_along * e.along;
    if (v_along < 0.0) {                       // dock 쪽으로 들어가는 것은 깔때기 안에 있을 때만
      v_along *= gate;
    }
    const double v_lat = -p.k_lat * e.lat;
    // 로봇 좌표계로
    const double c = std::cos(robot_yaw_in_dock), s = std::sin(robot_yaw_in_dock);
    want.vx = c * v_along + s * v_lat;
    want.vy = -s * v_along + c * v_lat;
    // 축마다 최대 속도. 방향(dock 좌표계에서 곧은 길)은 지킨다
    const double ratio = std::max({std::fabs(want.vx) / p.max_vx, std::fabs(want.vy) / p.max_vy, 1.0});
    want.vx /= ratio;
    want.vy /= ratio;
    want.wz = std::clamp(-p.k_yaw * e.yaw, -p.max_wz, p.max_wz);
    // 최소 속도: 로봇 축으로 남은 거리가 허용오차 밖이면 데드밴드 밑으로 내리지 않는다.
    // 단 깔때기가 닫혀 있으면(gate 0) dock 쪽으로 미는 최소 속도는 내지 않는다
    const double need_x = -(c * e.along + s * e.lat);          // 로봇 좌표계로 더 가야 할 거리
    const double need_y = -(-s * e.along + c * e.lat);
    const bool inward_x = need_x * c < 0.0;                    // 로봇 x 로 가면 dock 에 가까워진다
    if (std::fabs(need_x) > p.tol_along && std::fabs(want.vx) < p.min_vx && !(inward_x && gate <= 0.0)) {
      want.vx = std::copysign(p.min_vx, need_x);
    }
    if (std::fabs(need_y) > p.tol_lat && std::fabs(want.vy) < p.min_vy) {
      want.vy = std::copysign(p.min_vy, need_y);
    }
    if (std::fabs(e.yaw) > p.tol_yaw && std::fabs(want.wz) < p.min_wz) {
      want.wz = std::copysign(p.min_wz, -e.yaw);
    }
    // 명령 배율 (실제 로봇에서 잰 미끄러짐을 미리 보정)
    want.vx *= p.scale_vx;
    want.vy *= p.scale_vy;
    want.wz *= p.scale_wz;
  }
  auto limit = [dt](double target, double previous, double accel) {
      return std::clamp(target, previous - accel * dt, previous + accel * dt);
    };
  return {limit(want.vx, last.vx, p.accel_x), limit(want.vy, last.vy, p.accel_y),
    limit(want.wz, last.wz, p.accel_wz)};
}

}  // namespace aruco_dock
}  // namespace bt_mission

#endif  // BT_MISSION__ARUCO_DOCK_CONTROL_HPP_
