#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "straight_line_planner/line_builder.hpp"

using straight_line_planner::buildLine;
using straight_line_planner::firstBlockedDistance;
using straight_line_planner::Pose2D;

namespace
{
const Pose2D kStart{0.0, 0.0, 0.3};
const Pose2D kGoal{2.0, 2.0, 1.0};
constexpr double kRes = 0.1;
}  // namespace

TEST(LineBuilder, P0OnlyEndpoints)
{
  const auto path = buildLine(kStart, kGoal, kRes, 0);
  ASSERT_EQ(path.size(), 2u);
  EXPECT_DOUBLE_EQ(path.front().yaw, kStart.yaw);
  EXPECT_DOUBLE_EQ(path.back().yaw, kGoal.yaw);
}

TEST(LineBuilder, P1InterpolatesButLeavesYawZero)
{
  const auto path = buildLine(kStart, kGoal, kRes, 1);
  // int(2.828 / 0.1) = 28 개 + goal
  ASSERT_EQ(path.size(), 29u);
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    EXPECT_DOUBLE_EQ(path[i].yaw, 0.0) << "pose " << i;
  }
  EXPECT_DOUBLE_EQ(path.back().yaw, kGoal.yaw);
}

TEST(LineBuilder, P1ShorterThanResolutionKeepsOnlyGoal)
{
  // 공식 튜토리얼 코드의 동작: n = 0 -> goal 한 점. NaN 은 경로에 들어가지 않는다.
  const auto path = buildLine(kStart, Pose2D{0.05, 0.0, 0.0}, kRes, 1);
  ASSERT_EQ(path.size(), 1u);
  EXPECT_FALSE(std::isnan(path[0].x));
}

TEST(LineBuilder, P2OrientsPosesAlongTheLine)
{
  const auto path = buildLine(kStart, kGoal, kRes, 2);
  const double heading = std::atan2(2.0, 2.0);
  ASSERT_GE(path.size(), 3u);
  EXPECT_DOUBLE_EQ(path.front().x, kStart.x);
  EXPECT_DOUBLE_EQ(path.front().y, kStart.y);
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    EXPECT_NEAR(path[i].yaw, heading, 1e-12);
  }
  for (size_t i = 1; i < path.size(); ++i) {
    EXPECT_LE(std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), kRes + 1e-9);
  }
  EXPECT_DOUBLE_EQ(path.back().yaw, kGoal.yaw);
}

TEST(LineBuilder, P2StartEqualsGoal)
{
  const auto path = buildLine(kGoal, kGoal, kRes, 2);
  ASSERT_EQ(path.size(), 2u);
  for (const auto & p : path) {
    EXPECT_FALSE(std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.yaw));
  }
}

TEST(LineBuilder, FirstBlockedDistance)
{
  // (1, 0) 에 반지름 0.3 인 원형 장애물
  auto blocked = [](double x, double y) {return std::hypot(x - 1.0, y) < 0.3;};
  const double d = firstBlockedDistance({0, 0, 0}, {2, 0, 0}, 0.01, blocked);
  EXPECT_NEAR(d, 0.7, 0.011);
  EXPECT_LT(firstBlockedDistance({0, 1, 0}, {2, 1, 0}, 0.01, blocked), 0.0);
}
