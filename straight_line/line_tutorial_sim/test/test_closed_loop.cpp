// 튜토리얼 README 에 적은 '예상 결과'를 시뮬레이터로 검증한다.
// 제어 법칙이나 경로 생성 코드를 바꿨는데 여기서 깨지면, README 의 설명도 같이 고쳐야 한다.

#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "line_tutorial_sim/kinematic_sim.hpp"

using line_tutorial_sim::defaultScenarios;
using line_tutorial_sim::Scenario;
using line_tutorial_sim::SimConfig;
using line_tutorial_sim::SimResult;
using line_tutorial_sim::simulate;

namespace
{

const Scenario & scenario(const std::string & name)
{
  static const auto all = defaultScenarios();
  for (const auto & s : all) {
    if (s.name == name) {
      return s;
    }
  }
  throw std::runtime_error("unknown scenario " + name);
}

SimResult run(const std::string & name, int planner, int controller, bool replan = false)
{
  SimConfig cfg;
  cfg.planner_step = planner;
  cfg.controller_step = controller;
  cfg.replan = replan;
  return simulate(scenario(name), cfg);
}

const std::vector<std::string> kOpen{"S1_straight", "S2_behind", "S4_goal_yaw", "S5_offset"};

}  // namespace

TEST(Planner, FinalPairReachesEveryOpenScenario)
{
  for (const auto & s : kOpen) {
    EXPECT_TRUE(run(s, 2, 4).reached()) << s;
    EXPECT_TRUE(run(s, 2, 4, true).reached()) << s << " (replan)";
  }
}

TEST(Planner, P0SparsePathStallsPurePursuit)
{
  // 두 점짜리 경로: goal 이 local costmap(1.5m) 밖이라 잘려 나가고 carrot 이 로봇 위에 남는다
  for (const auto & s : kOpen) {
    EXPECT_EQ(run(s, 0, 4).outcome, "no_progress") << s;
  }
  // 경로를 안 쓰는 C0 는 상관없다
  EXPECT_TRUE(run("S1_straight", 0, 0).reached());
}

TEST(Planner, P3RejectsBlockedLineOthersCollide)
{
  EXPECT_EQ(run("S3_pillar", 3, 4).outcome, "plan_rejected");
  EXPECT_EQ(run("S3_pillar", 2, 4).outcome, "collision");
  EXPECT_EQ(run("S3_pillar", 1, 4).outcome, "collision");
}

TEST(Planner, P1OrientationBreaksHeadingTrackOnly)
{
  // C1 은 경로의 orientation 을 쓰므로 P1(중간 yaw = 0)에서 무너지고 P2 에서 고쳐진다
  const auto p1 = run("S5_offset", 1, 1);
  EXPECT_EQ(p1.outcome, "controller_failed");
  EXPECT_GT(p1.max_cte, 0.5);
  EXPECT_TRUE(run("S5_offset", 2, 1).reached());
  // pure pursuit 는 orientation 을 쓰지 않으므로 P1 이어도 된다
  EXPECT_TRUE(run("S5_offset", 1, 4).reached());
  // map +x 방향 직선에서는 yaw = 0 이 우연히 맞으므로 결함이 드러나지 않는다
  EXPECT_TRUE(run("S1_straight", 1, 1).reached());
}

TEST(Controller, PurePursuitFailsWhenCarrotIsBehind)
{
  // goal 이 등 뒤: 곡률이 거의 0 이라 앞으로 가 버린다
  EXPECT_EQ(run("S2_behind", 2, 2).outcome, "controller_failed");
  // goal yaw 가 다르면 goal 을 지나친 뒤 같은 이유로 돌아오지 못한다
  EXPECT_EQ(run("S4_goal_yaw", 2, 2).outcome, "controller_failed");
  // C3 의 제자리 회전이 둘 다 고친다
  EXPECT_TRUE(run("S2_behind", 2, 3).reached());
  EXPECT_TRUE(run("S4_goal_yaw", 2, 3).reached());
}

TEST(Controller, LimitsBoundCommandedAcceleration)
{
  const SimConfig cfg;
  const auto c3 = run("S1_straight", 2, 3);
  const auto c4 = run("S1_straight", 2, 4);
  EXPECT_GT(c3.peak_cmd_accel, 4.0);      // 0 -> 0.25 m/s 한 주기에 점프
  EXPECT_LE(c4.peak_cmd_accel, cfg.controller.max_linear_accel + 1e-9);
  EXPECT_LT(c4.peak_cmd_decel, c3.peak_cmd_decel);   // goal 접근 감속 덕분에 정지 충격이 작다
}

TEST(Replanning, TurnsControllerFailureIntoCrash)
{
  // 한 번만 계획하면 컨트롤러가 분명하게 실패한다
  const auto once = run("S4_goal_yaw", 2, 2, false);
  EXPECT_EQ(once.outcome, "controller_failed");

  // 1Hz 재계획이면 경로가 매초 로봇 위치에서 다시 시작하므로 컨트롤러 실패가 사라진다.
  // '최신 경로 기준' 오차는 1s x 0.25m/s 이하로 작게 보이지만, 로봇은 그대로 벽까지 가 버린다
  const auto replan = run("S4_goal_yaw", 2, 2, true);
  EXPECT_EQ(replan.outcome, "collision");
  EXPECT_LE(replan.max_cte_latest, 0.26);
  EXPECT_GT(replan.max_cte, 1.5);
}

TEST(Replanning, IncreasesDeviationFromOriginalLine)
{
  const auto once = run("S5_offset", 2, 4, false);
  const auto replan = run("S5_offset", 2, 4, true);
  ASSERT_TRUE(once.reached());
  ASSERT_TRUE(replan.reached());
  EXPECT_GT(replan.rms_cte, once.rms_cte);
  EXPECT_LT(replan.max_cte_latest, replan.max_cte);
}
