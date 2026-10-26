// motion_profile.hpp: S 커브 가감속과 브레이크 거리
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "simple_line_controller/motion_profile.hpp"

using simple_line_controller::brakingDistance;
using simple_line_controller::Motion;
using simple_line_controller::MotionLimits;
using simple_line_controller::stepToward;

namespace
{

constexpr double kDt = 0.05;

// 목표 속도까지 stepToward 를 반복한 기록
std::vector<Motion> run(Motion m, double target, const MotionLimits & lim, int steps = 400)
{
  std::vector<Motion> out{m};
  for (int i = 0; i < steps; ++i) {
    out.push_back(m = stepToward(m, target, lim, kDt));
  }
  return out;
}

void expectWithinLimits(const std::vector<Motion> & h, const MotionLimits & lim)
{
  for (size_t i = 1; i < h.size(); ++i) {
    EXPECT_LE(h[i].a, lim.accel + 1e-9) << "step " << i;
    EXPECT_GE(h[i].a, -lim.decel - 1e-9) << "step " << i;
    if (lim.jerk > 0.0) {
      EXPECT_LE(std::fabs(h[i].a - h[i - 1].a), lim.jerk * kDt + 1e-9) << "step " << i;
    }
  }
}

}  // namespace

TEST(MotionProfile, AcceleratesSmoothlyToTheTarget)
{
  const MotionLimits lim{0.5, 0.5, 1.0};
  const auto h = run({0.0, 0.0}, 0.25, lim);
  expectWithinLimits(h, lim);
  for (size_t i = 1; i < h.size(); ++i) {
    EXPECT_GE(h[i].v, h[i - 1].v - 1e-12);   // 줄지 않는다
    EXPECT_LE(h[i].v, 0.25 + 1e-12);         // 넘지 않는다
  }
  EXPECT_DOUBLE_EQ(h.back().v, 0.25);
  EXPECT_DOUBLE_EQ(h.back().a, 0.0);
  // 0.25 m/s 를 가속도 0.5, jerk 1 로: 가속도가 0 → 0.5 → 0 인 삼각형, 약 1초
  int reached = 0;
  while (h[reached].v < 0.25) {
    ++reached;
  }
  EXPECT_NEAR(reached * kDt, 1.0, 0.11);
}

TEST(MotionProfile, DeceleratesSmoothlyToStop)
{
  const MotionLimits lim{0.5, 0.5, 1.0};
  const auto h = run({0.25, 0.0}, 0.0, lim);
  expectWithinLimits(h, lim);
  EXPECT_DOUBLE_EQ(h.back().v, 0.0);
  EXPECT_DOUBLE_EQ(h.back().a, 0.0);
}

TEST(MotionProfile, WithoutJerkLimitItIsATrapezoid)
{
  const MotionLimits lim{0.5, 0.4, 0.0};
  const auto up = run({0.0, 0.0}, 0.25, lim, 20);
  EXPECT_NEAR(up[1].v, 0.5 * kDt, 1e-12);     // 첫 주기부터 최대 가속
  EXPECT_DOUBLE_EQ(up.back().v, 0.25);
  const auto down = run({0.25, 0.0}, 0.0, lim, 20);
  EXPECT_NEAR(down[1].v, 0.25 - 0.4 * kDt, 1e-12);
  EXPECT_DOUBLE_EQ(down.back().v, 0.0);
}

TEST(MotionProfile, ChangesDirectionOfAccelerationSmoothly)
{
  // 가속 중에 목표가 낮아져도 가속도가 jerk 로 제한되어 바뀐다
  const MotionLimits lim{0.5, 0.5, 1.0};
  Motion m{0.1, 0.4};
  std::vector<Motion> h{m};
  for (int i = 0; i < 100; ++i) {
    h.push_back(m = stepToward(m, 0.05, lim, kDt));
  }
  expectWithinLimits(h, lim);
  EXPECT_NEAR(h.back().v, 0.05, 1e-9);
}

TEST(MotionProfile, BrakingDistanceMatchesTheSCurve)
{
  // 0.25 m/s 에서 감속도 0.5, jerk 1 로 멈추는 거리 (연속 시간): v^2/(2a) + v·a/(2j) = 0.125 m
  const MotionLimits lim{0.5, 0.5, 1.0};
  EXPECT_NEAR(brakingDistance({0.25, 0.0}, 0.0, lim, kDt), 0.125, 0.01);
  // jerk 제한이 없으면 v^2/(2a) = 0.0625 m
  EXPECT_NEAR(brakingDistance({0.25, 0.0}, 0.0, {0.5, 0.5, 0.0}, kDt), 0.0625, 0.01);
  // 가속 중이면 가속도를 먼저 되돌려야 해서 더 간다
  EXPECT_GT(brakingDistance({0.25, 0.4}, 0.0, lim, kDt), 0.125);
  // 이미 목표 이하로 서 있으면 0
  EXPECT_DOUBLE_EQ(brakingDistance({0.0, 0.0}, 0.05, lim, kDt), 0.0);
}

TEST(MotionProfile, BrakingWhenTheDistanceRunsOutStopsAtThatPoint)
{
  // 컨트롤러와 같은 방식: 브레이크 거리가 남은 거리에 닿으면 감속. 로봇은 명령을 0.1초 늦게 따른다
  const MotionLimits lim{0.5, 0.5, 1.0};
  for (double dist : {0.12, 0.5, 2.0}) {
    Motion m;
    double x = 0.0;
    std::vector<double> delay(2, 0.0);
    bool braking = false;
    for (int i = 0; i < 1000; ++i) {
      const double d = dist - x;
      double target = 0.0;
      if (d > 0.0) {
        const double need = brakingDistance(m, 0.03, lim, kDt);
        braking = braking || need >= d - m.v * kDt;
        target = braking ? 0.03 : 0.25;
      }
      m = stepToward(m, target, lim, kDt);
      delay.push_back(m.v);
      x += delay.front() * kDt;
      delay.erase(delay.begin());
    }
    // 남은 거리 0 에서 0.03 m/s 로 다가가다 서는 거리와 0.1초 지연만큼 조금 지나간다 (컨트롤러는 도착
    // 범위 0.05 m 앞을 이 지점으로 잡으므로 goal 은 지나가지 않는다)
    EXPECT_GE(x, dist - 0.005) << "dist " << dist;
    EXPECT_LE(x, dist + 0.02) << "dist " << dist;
  }
}
