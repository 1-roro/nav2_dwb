// ArUco 도킹 계산 (aruco_dock_control.hpp) 시험. ROS 없이 계산만 본다
#include <cmath>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "bt_mission/aruco_dock_control.hpp"

namespace ad = bt_mission::aruco_dock;
using ad::Cmd;
using ad::DockErrors;
using ad::MecanumParams;
using ad::Pose2;

TEST(ArucoDockControl, PoseAlgebra)
{
  const Pose2 dock{4.55, 1.0, M_PI};                 // dock 좌표계: 방 쪽(-x 월드)을 본다
  const Pose2 robot{4.45, 1.02, 0.0};                // dock 앞 0.10 m, 옆 0.02 m, dock 을 바라봄
  const Pose2 r = ad::relative(dock, robot);
  EXPECT_NEAR(r.x, 0.10, 1e-12);
  EXPECT_NEAR(r.y, -0.02, 1e-12);                    // 월드 +y 는 dock 좌표계 -y
  EXPECT_NEAR(std::fabs(r.yaw), M_PI, 1e-12);
  const Pose2 back = ad::compose(dock, r);
  EXPECT_NEAR(back.x, robot.x, 1e-12);
  EXPECT_NEAR(back.y, robot.y, 1e-12);
}

TEST(ArucoDockControl, FilterWeightsPreciseMeasurementsMore)
{
  ad::DockFilter f;
  const Pose2 truth{4.55, 1.0, M_PI};
  f.reset({4.58, 1.03, M_PI + 0.03}, ad::diag3(0.03 * 0.03, 0.03 * 0.03, 0.03 * 0.03));   // 멀리서 본 거친 값
  // 가까이서 본 정확한 측정 (1 mm, 0.2 도) 몇 개면 거기로 간다
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int i = 0; i < 10; ++i) {
    f.predict(0.07, 0.002, 0.002);
    const auto r = f.update({truth.x + 0.001 * n(rng), truth.y + 0.001 * n(rng), truth.yaw + 0.0035 * n(rng)},
      ad::diag3(1e-6, 1e-6, 0.0035 * 0.0035), 16.3, 8);
    EXPECT_EQ(r, ad::DockFilter::Result::Accepted);
  }
  EXPECT_NEAR(f.mean().x, truth.x, 0.001);
  EXPECT_NEAR(f.mean().y, truth.y, 0.001);
  EXPECT_NEAR(ad::normalizeAngle(f.mean().yaw - truth.yaw), 0.0, 0.003);
  EXPECT_LT(std::sqrt(f.cov()[0]), 0.001);
}

TEST(ArucoDockControl, FilterRejectsAnOutlierAndReinitializesAfterManyInARow)
{
  ad::DockFilter f;
  f.reset({4.55, 1.0, M_PI}, ad::diag3(1e-6, 1e-6, 1e-5));
  const auto r = ad::diag3(1e-6, 1e-6, 1e-5);
  EXPECT_EQ(f.update({4.55, 1.0, M_PI + 0.3}, r, 16.3, 3), ad::DockFilter::Result::Rejected);  // 뒤집힌 해
  EXPECT_NEAR(f.mean().yaw, M_PI, 1e-9);
  EXPECT_EQ(f.update({4.60, 1.0, M_PI}, r, 16.3, 3), ad::DockFilter::Result::Rejected);
  // 처음 값이 틀렸다면 바른 측정이 계속 버려진다: 세 번째에 다시 잡는다
  EXPECT_EQ(f.update({4.60, 1.0, M_PI}, r, 16.3, 3), ad::DockFilter::Result::Reinitialized);
  EXPECT_NEAR(f.mean().x, 4.60, 1e-9);
}

TEST(ArucoDockControl, FuseDropsAFlippedDetection)
{
  std::vector<ad::Measurement> ms;
  for (int i = 0; i < 9; ++i) {
    ms.push_back({{4.55 + 0.0005 * (i % 3), 1.0, -M_PI + 0.001 * (i % 2)}, ad::diag3(1e-6, 1e-6, 4e-6)});
  }
  ms.push_back({{4.56, 1.01, M_PI - 0.25}, ad::diag3(1e-6, 1e-6, 4e-6)});   // 뒤집힘 (±pi 넘김도 섞음)
  int used = 0;
  const auto m = ad::fuse(ms, 16.3, &used);
  ASSERT_TRUE(m);
  EXPECT_EQ(used, 9);
  EXPECT_NEAR(m->pose.x, 4.5505, 0.0003);
  EXPECT_NEAR(std::fabs(m->pose.yaw), M_PI, 0.002);
  EXPECT_LT(m->cov[0], 1e-6);                        // 9개를 합쳐 하나보다 작다
}

TEST(ArucoDockControl, FunnelHoldsTheApproachUntilAligned)
{
  MecanumParams p;
  p.accel_x = p.accel_y = p.accel_wz = 100.0;        // 가속도 제한을 빼고 본다
  // dock 앞 0.2 m 에서 옆으로 5 cm: 폭 0.004 + 0.15*0.2 = 0.034 m 를 넘는다 -> 앞으로 가지 않고 옆만
  Cmd cmd = ad::mecanumStep({0.2, 0.05, 0.0}, M_PI, {}, p, 0.05);
  EXPECT_NEAR(cmd.vx, 0.0, 1e-9);
  EXPECT_GT(cmd.vy, 0.0);                            // dock 좌표계 +y 로 벗어남 = 로봇의 오른쪽 -> 왼쪽(+y)으로
  // 옆이 맞으면 앞으로 (로봇 +x = dock 쪽)
  cmd = ad::mecanumStep({0.2, 0.002, 0.0}, M_PI, {}, p, 0.05);
  EXPECT_GT(cmd.vx, 0.1);
  // 지나쳤으면(along < 0) 깔때기와 상관없이 뒤로
  cmd = ad::mecanumStep({-0.01, 0.05, 0.0}, M_PI, {}, p, 0.05);
  EXPECT_LT(cmd.vx, 0.0);
}

TEST(ArucoDockControl, PerAxisLimitsKeepTheDirectionAndMinimumSpeeds)
{
  MecanumParams p;
  p.accel_x = p.accel_y = p.accel_wz = 100.0;
  p.funnel_lat_slope = 10.0;                         // 깔때기를 열어 둔다
  const Cmd cmd = ad::mecanumStep({0.5, 0.3, 0.0}, M_PI, {}, p, 0.05);
  EXPECT_LE(std::fabs(cmd.vx), p.max_vx + 1e-12);
  EXPECT_LE(std::fabs(cmd.vy), p.max_vy + 1e-12);
  EXPECT_NEAR(cmd.vy / cmd.vx, (1.2 * 0.3) / (0.8 * 0.5), 1e-9);   // 곧은 길 유지 (k_lat/k_along 비율)
  // 남은 옆 오차 4 mm: P 로는 4.8 mm/s 지만 최소 속도로 (메카넘 옆 데드밴드)
  const Cmd small = ad::mecanumStep({0.0, 0.004, 0.0}, M_PI, {}, p, 0.05);
  EXPECT_NEAR(std::fabs(small.vy), p.min_vy, 1e-12);
  // 명령 배율: 옆으로 85 % 만 나오는 로봇이면 1/0.85 배로 낸다
  p.scale_vy = 1.0 / 0.85;
  const Cmd scaled = ad::mecanumStep({0.0, 0.004, 0.0}, M_PI, {}, p, 0.05);
  EXPECT_NEAR(std::fabs(scaled.vy), p.min_vy / 0.85, 1e-12);
}

// 메카넘의 미끄러짐(옆 85 %, 옆으로 가면 조금 돈다), 한 주기 늦음, 측정 잡음이 있을 때 들어가는지
TEST(ArucoDockControl, DocksThroughSlipAndLatencyWithoutLeavingTheFunnel)
{
  MecanumParams p;
  const Pose2 dock{4.55, 1.0, M_PI};
  const Pose2 target{0.10, 0.0, M_PI};               // dock 앞 0.10 m 에서 dock 을 본다
  Pose2 robot = ad::compose(dock, {0.55, -0.12, M_PI + 0.15});   // 0.45 m 앞, 옆 12 cm, 8.6 도
  std::mt19937 rng(2);
  std::normal_distribution<double> n(0.0, 1.0);
  ad::DockFilter f;
  f.reset({dock.x + 0.004, dock.y - 0.003, dock.yaw + 0.01}, ad::diag3(1e-5, 1e-5, 1e-4));
  const double dt = 0.05;
  Cmd cmd, applied;
  double worst_outside_funnel = 0.0;
  double farthest_in = 1e9;
  int t = 0;
  for (; t < 1200; ++t) {
    // 측정 (가까울수록 정확하다)
    const double dist = ad::relative(dock, robot).x + 0.3;
    const double sigma = 0.0005 * dist * dist / 0.1;
    f.predict(dt, 0.002, 0.002);
    f.update({dock.x + sigma * n(rng), dock.y + sigma * n(rng), dock.yaw + 3.0 * sigma * n(rng)},
      ad::diag3(sigma * sigma, sigma * sigma, 9 * sigma * sigma), 16.3, 8);
    const Pose2 in_dock = ad::relative(f.mean(), robot);
    const DockErrors e = ad::dockErrors(in_dock, target);
    if (ad::withinTolerance(e, p) && std::hypot(applied.vx, applied.vy) < 1e-6) {
      break;
    }
    cmd = ad::mecanumStep(e, in_dock.yaw, cmd, p, dt);
    // 로봇: 옆 85 %, 옆으로 1 m 가면 0.05 rad 돈다
    const Cmd body{applied.vx, 0.85 * applied.vy, applied.wz + 0.05 * applied.vy};
    robot = ad::compose(robot, {body.vx * dt, body.vy * dt, body.wz * dt});
    applied = cmd;
    const DockErrors truth = ad::dockErrors(ad::relative(dock, robot), target);
    const double band = p.funnel_lat0 + p.funnel_lat_slope * std::max(truth.along, 0.0);
    if (truth.along < 0.15) {
      worst_outside_funnel = std::max(worst_outside_funnel, std::fabs(truth.lat) - band);
    }
    farthest_in = std::min(farthest_in, truth.along);
  }
  const DockErrors final = ad::dockErrors(ad::relative(dock, robot), target);
  EXPECT_LT(t, 1200) << "did not settle";
  EXPECT_LT(std::fabs(final.along), 0.004);
  EXPECT_LT(std::fabs(final.lat), 0.003);
  EXPECT_LT(std::fabs(final.yaw), 0.008);
  EXPECT_LE(worst_outside_funnel, 0.002) << "entered the last 15 cm outside the funnel";
  EXPECT_GT(farthest_in, -0.004) << "pushed into the dock";
}
