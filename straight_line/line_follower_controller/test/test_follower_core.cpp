#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "line_follower_controller/follower_core.hpp"

using namespace line_follower_controller;  // NOLINT

namespace
{
constexpr double kDt = 0.05;
constexpr double kTol = 0.25;

// 로봇 좌표계에서 (x0, y0) -> (x1, y1) 직선, 간격 0.1m, 모든 pose 의 yaw = yaw
std::vector<Pose2D> line(double x0, double y0, double x1, double y1, double yaw)
{
  std::vector<Pose2D> out;
  const double d = std::hypot(x1 - x0, y1 - y0);
  const int n = static_cast<int>(std::ceil(d / 0.1));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    out.push_back({x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw});
  }
  return out;
}

Params withStep(int step)
{
  Params p;
  p.step = step;
  return p;
}
}  // namespace

TEST(FollowerCore, ToFrame)
{
  const Pose2D robot{1.0, 1.0, M_PI / 2};
  const Pose2D p = toFrame(robot, {1.0, 2.0, M_PI / 2});
  EXPECT_NEAR(p.x, 1.0, 1e-12);   // 로봇 정면 1m
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  EXPECT_NEAR(p.yaw, 0.0, 1e-12);
}

TEST(FollowerCore, LocalWindowDropsGoalOfTwoPosePath)
{
  // P0 경로: [start, 5m 앞 goal]. local costmap 반폭 1.5m -> start 만 남는다.
  const std::vector<Pose2D> plan{{0, 0, 0}, {5, 0, 0}};
  const auto local = localWindow(plan, 0, {0, 0, 0}, 1.5);
  ASSERT_EQ(local.size(), 1u);
  EXPECT_NEAR(local[0].x, 0.0, 1e-12);

  // carrot 이 로봇 위에 있으니 pure pursuit 는 움직이지 않는다
  const auto cmd = computeCommand(withStep(PURE_PURSUIT), local, {5, 0, 0}, {}, kDt, kTol);
  EXPECT_STREQ(cmd.state, "carrot_at_robot");
  EXPECT_DOUBLE_EQ(cmd.v, 0.0);
}

TEST(FollowerCore, EmptyLocalPlanIsAnError)
{
  const auto cmd = computeCommand(withStep(PURE_PURSUIT), {}, {5, 0, 0}, {}, kDt, kTol);
  EXPECT_FALSE(cmd.ok);
  // C0 는 경로를 쓰지 않으므로 상관없다
  EXPECT_TRUE(computeCommand(withStep(GOAL_SEEK), {}, {5, 0, 0}, {}, kDt, kTol).ok);
}

TEST(FollowerCore, HeadingTrackFollowsPathOrientation)
{
  // 경로는 로봇 정면(+x)으로 뻗어 있는데 orientation 이 -90도로 잘못 들어온 경우 (P1 경로)
  const auto bad = line(0, 0, 1.5, 0, -M_PI / 2);
  const auto cmd = computeCommand(withStep(HEADING_TRACK), bad, {3, 0, 0}, {}, kDt, kTol);
  EXPECT_LT(cmd.w, -0.5);   // 경로가 정면인데도 오른쪽으로 크게 꺾는다

  const auto good = line(0, 0, 1.5, 0, 0.0);
  EXPECT_NEAR(
    computeCommand(withStep(HEADING_TRACK), good, {3, 0, 0}, {}, kDt, kTol).w, 0.0, 1e-9);
}

TEST(FollowerCore, PurePursuitIgnoresGoalStraightBehind)
{
  // 경로가 정확히 등 뒤로 뻗어 있으면 carrot.y = 0 -> 곡률 0 -> 그대로 앞으로 간다
  const auto behind = line(0, 0, -1.5, 0, M_PI);
  const auto cmd = computeCommand(withStep(PURE_PURSUIT), behind, {-3, 0, M_PI}, {}, kDt, kTol);
  EXPECT_GT(cmd.v, 0.0);
  EXPECT_NEAR(cmd.w, 0.0, 1e-9);

  // C3 는 먼저 제자리에서 돈다
  const auto fixed = computeCommand(withStep(ROTATE), behind, {-3, 0, M_PI}, {}, kDt, kTol);
  EXPECT_STREQ(fixed.state, "rotate_to_path");
  EXPECT_DOUBLE_EQ(fixed.v, 0.0);
}

TEST(FollowerCore, RotateToGoalYawInsideTolerance)
{
  const std::vector<Pose2D> local{{0.1, 0.0, M_PI / 2}};
  const auto cmd = computeCommand(withStep(ROTATE), local, {0.1, 0.0, M_PI / 2}, {}, kDt, kTol);
  EXPECT_STREQ(cmd.state, "rotate_to_goal");
  EXPECT_DOUBLE_EQ(cmd.v, 0.0);
  EXPECT_GT(cmd.w, 0.0);
}

TEST(FollowerCore, LimitsRampUpFromStandstill)
{
  const auto ahead = line(0, 0, 1.5, 0, 0.0);
  const Params p = withStep(LIMITS);
  const auto cmd = computeCommand(p, ahead, {3, 0, 0}, {0.0, 0.0}, kDt, kTol);
  EXPECT_NEAR(cmd.v, p.max_linear_accel * kDt, 1e-12);

  const auto c2 = computeCommand(withStep(PURE_PURSUIT), ahead, {3, 0, 0}, {0.0, 0.0}, kDt, kTol);
  EXPECT_DOUBLE_EQ(c2.v, Params{}.desired_linear_vel);   // C2 는 0 -> 0.25 m/s 로 바로 점프
}

TEST(FollowerCore, AngularLimitKeepsCurvature)
{
  // 왼쪽으로 급하게 꺾인 경로: w 가 상한에 걸리면 v 도 같이 줄어 곡률이 유지돼야 한다
  const std::vector<Pose2D> local{{0.3, 0.3, M_PI / 2}, {0.3, 0.6, M_PI / 2}};
  Params p = withStep(PURE_PURSUIT);
  p.desired_linear_vel = 0.5;   // w = 0.5 * 2.67 = 1.33 > max_angular_vel(1.0)
  const auto cmd = computeCommand(p, local, {0.3, 0.6, M_PI / 2}, {}, kDt, kTol);
  EXPECT_NEAR(std::fabs(cmd.w), p.max_angular_vel, 1e-12);
  const double L2 = 0.3 * 0.3 + 0.6 * 0.6;
  EXPECT_NEAR(cmd.w / cmd.v, 2.0 * 0.6 / L2, 1e-9);
}
