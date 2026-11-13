// 도킹에 같이 쓰는 계산 (dock_control.hpp: BackOff 의 제어, 뒤의 레이저 점) 시험. ROS 없이 계산만 본다
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "bt_mission/dock_control.hpp"

using bt_mission::dock::Cmd;
using bt_mission::dock::ControlParams;
using bt_mission::dock::Pose2;
namespace dock = bt_mission::dock;

TEST(DockControl, ErrorIsInTheRobotFrame)
{
  // 로봇이 +y 를 보고 있으면, world 의 +y 쪽 목표는 로봇의 앞(+x), world 의 -x 쪽 목표는 로봇의 오른쪽(-y)
  const Pose2 robot{1.0, 2.0, M_PI / 2};
  auto e = dock::errorInRobotFrame(robot, {1.0, 3.0, M_PI / 2});
  EXPECT_NEAR(e.x, 1.0, 1e-9);
  EXPECT_NEAR(e.y, 0.0, 1e-9);
  e = dock::errorInRobotFrame(robot, {2.0, 2.0, 0.0});
  EXPECT_NEAR(e.x, 0.0, 1e-9);
  EXPECT_NEAR(e.y, -1.0, 1e-9);
  EXPECT_NEAR(e.yaw, -M_PI / 2, 1e-9);
}

TEST(DockControl, LinearSpeedKeepsItsDirectionWhenLimited)
{
  ControlParams p;
  p.lin_accel = 100.0;                  // 가속도 제한을 빼고 본다
  const Cmd cmd = dock::holonomicStep({0.4, 0.3, 0.0}, {}, p, 0.05);
  EXPECT_NEAR(std::hypot(cmd.vx, cmd.vy), p.max_lin, 1e-9);
  EXPECT_NEAR(cmd.vy / cmd.vx, 0.3 / 0.4, 1e-9);   // 대각선 그대로 (옆으로 가는 비율 유지)
}

TEST(DockControl, AccelerationIsLimitedOnEachAxis)
{
  ControlParams p;
  const Cmd cmd = dock::holonomicStep({1.0, -1.0, 1.0}, {}, p, 0.05);
  EXPECT_NEAR(cmd.vx, p.lin_accel * 0.05, 1e-9);
  EXPECT_NEAR(cmd.vy, -p.lin_accel * 0.05, 1e-9);
  EXPECT_NEAR(cmd.wz, p.ang_accel * 0.05, 1e-9);
}

TEST(DockControl, MinimumSpeedOutsideToleranceAndZeroInside)
{
  ControlParams p;
  p.lin_accel = p.ang_accel = 100.0;
  Cmd cmd = dock::holonomicStep({0.012, 0.0, 0.0}, {}, p, 0.05);    // P 만으로는 0.012 m/s
  EXPECT_NEAR(cmd.vx, std::max(p.min_lin, p.k_lin * 0.012), 1e-9);
  cmd = dock::holonomicStep({0.005, 0.0, 0.01}, {}, p, 0.05);       // 허용오차 안
  EXPECT_EQ(cmd.vx, 0.0);
  EXPECT_EQ(cmd.wz, 0.0);
}

TEST(DockControl, PointsBehindAreThoseInTheSweptRectangle)
{
  // 뒤 끝 0.2 m, 폭 ±0.16 m 인 로봇이 0.2 m 물러날 때: x -0.2 ~ -0.45, y ±0.21 (여유 0.05)
  const std::vector<dock::Point2> points{
    {-0.35, 0.0}, {-0.44, 0.2},     // 안
    {-0.5, 0.0}, {-0.3, 0.3},       // 너무 멀다, 옆으로 비켜 있다
    {-0.1, 0.0}, {0.5, 0.0}};       // 몸 안(레이저 잡음), 앞
  EXPECT_EQ(dock::pointsBehind(points, 0.2, 0.16, 0.2, 0.05), 2);
}

// 한 주기 늦게 따라오는 홀로노믹 로봇을 굴려서 목표에 서는지 본다
static Pose2 simulate(Pose2 robot, const Pose2 & goal, const ControlParams & p, double seconds, double * max_speed)
{
  const double dt = 0.05;
  Cmd cmd, applied;
  for (int i = 0; i < static_cast<int>(seconds / dt); ++i) {
    const Pose2 error = dock::errorInRobotFrame(robot, goal);
    cmd = dock::holonomicStep(error, cmd, p, dt);
    // 로봇 좌표계의 속도를 world 로
    robot.x += (applied.vx * std::cos(robot.yaw) - applied.vy * std::sin(robot.yaw)) * dt;
    robot.y += (applied.vx * std::sin(robot.yaw) + applied.vy * std::cos(robot.yaw)) * dt;
    robot.yaw = dock::normalizeAngle(robot.yaw + applied.wz * dt);
    *max_speed = std::max(*max_speed, std::hypot(applied.vx, applied.vy));
    applied = cmd;
  }
  return robot;
}

TEST(DockControl, ConvergesFromALateralAndYawOffset)
{
  ControlParams p;
  const Pose2 goal{4.3, -1.5, 0.0};
  double max_speed = 0.0;
  const Pose2 end = simulate({3.9, -1.2, 15.0 * M_PI / 180.0}, goal, p, 15.0, &max_speed);
  const Pose2 error = dock::errorInRobotFrame(end, goal);
  EXPECT_TRUE(dock::withinTolerance(error, p)) << error.x << " " << error.y << " " << error.yaw;
  EXPECT_LE(max_speed, p.max_lin + 1e-9);
}

TEST(DockControl, DoesNotOvershootTheDock)
{
  // 앞으로만 0.4 m. 목표를 넘어가지 않는다
  ControlParams p;
  p.max_lin = 0.05;
  const Pose2 goal{4.7, -1.5, 0.0};
  Pose2 robot{4.3, -1.5, 0.0};
  const double dt = 0.05;
  Cmd cmd, applied;
  double farthest = robot.x;
  for (int i = 0; i < 400; ++i) {
    cmd = dock::holonomicStep(dock::errorInRobotFrame(robot, goal), cmd, p, dt);
    robot.x += applied.vx * dt;
    applied = cmd;
    farthest = std::max(farthest, robot.x);
  }
  EXPECT_NEAR(robot.x, goal.x, p.xy_tolerance);
  EXPECT_LE(farthest, goal.x + 0.003);                    // 한 주기 늦어도 3 mm 넘게 지나치지 않는다
}
