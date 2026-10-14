// planner/controller 단계 조합을 시나리오별로 돌려 표로 출력한다. (dwb_steps.py 의 출력 표와 같은 역할)
//
//   ros2 run line_tutorial_sim step_matrix                       # 튜토리얼 표 전체
//   ros2 run line_tutorial_sim step_matrix -s S5_offset -p 2 -c 2 --replan --trace c2.csv
//                                                                # 한 조합만 자세히 + 궤적 CSV

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "line_tutorial_sim/kinematic_sim.hpp"

using line_tutorial_sim::defaultScenarios;
using line_tutorial_sim::Scenario;
using line_tutorial_sim::SimConfig;
using line_tutorial_sim::SimResult;
using line_tutorial_sim::simulate;
namespace lf = line_follower_controller;
namespace slp = straight_line_planner;

namespace
{

std::string cell(const SimResult & r)
{
  char buf[64];
  if (r.reached()) {
    std::snprintf(buf, sizeof(buf), "ok %5.1fs", r.time);
  } else if (r.outcome == "controller_failed") {
    std::snprintf(buf, sizeof(buf), "ctrl_fail@%.1fs", r.time);
  } else if (r.outcome == "no_progress") {
    std::snprintf(buf, sizeof(buf), "no_progress");
  } else if (r.outcome == "plan_rejected") {
    std::snprintf(buf, sizeof(buf), "plan_reject");
  } else {
    std::snprintf(buf, sizeof(buf), "%s", r.outcome.c_str());
  }
  return buf;
}

void printHeader(const std::string & title, const std::vector<Scenario> & scs)
{
  std::printf("\n== %s\n", title.c_str());
  std::printf("%-22s", "");
  for (const auto & s : scs) {
    std::printf(" %-15s", s.name.c_str());
  }
  std::printf("\n");
}

void printRow(const std::string & label, const SimConfig & cfg, const std::vector<Scenario> & scs)
{
  std::printf("%-22s", label.c_str());
  for (const auto & s : scs) {
    std::printf(" %-15s", cell(simulate(s, cfg)).c_str());
  }
  std::printf("\n");
}

void printDetail(const std::string & label, const Scenario & sc, const SimConfig & cfg)
{
  const auto r = simulate(sc, cfg);
  std::printf(
    "%-30s %-17s %7.1f %8.2f %8.2f %8.2f %7.2f %8.2f %8.1f %8.1f\n", label.c_str(),
    r.outcome.c_str(), r.time, r.max_cte, r.rms_cte, r.max_cte_latest, r.final_xy_err,
    r.final_yaw_err, r.peak_cmd_accel, r.peak_cmd_decel);
}

void printDetailHeader(const std::string & title)
{
  std::printf("\n== %s\n", title.c_str());
  std::printf(
    "%-30s %-17s %7s %8s %8s %8s %7s %8s %8s %8s\n", "", "outcome", "time[s]", "maxCTE",
    "rmsCTE", "maxCTE*", "xy_err", "yaw_err", "peak_acc", "peak_dec");
}

std::string pc(int p, int c)
{
  return std::string(slp::stepName(p)).substr(0, 2) + " + " +
         std::string(lf::stepName(c)).substr(0, 2);
}

void tutorialTables()
{
  const auto scs = defaultScenarios();
  std::printf("scenarios (line_room map frame):\n");
  for (const auto & s : scs) {
    std::printf(
      "  %-12s start(%.1f, %.1f, %4.0fdeg) goal(%.1f, %.1f, %4.0fdeg)  %s\n", s.name.c_str(),
      s.start.x, s.start.y, s.start.yaw * 180 / M_PI, s.goal.x, s.goal.y,
      s.goal.yaw * 180 / M_PI, s.description.c_str());
  }

  SimConfig cfg;

  printHeader("A. planner steps, controller fixed to C4, plan once", scs);
  for (int p = slp::ENDPOINTS; p <= slp::COLLISION; ++p) {
    cfg.planner_step = p;
    cfg.controller_step = lf::LIMITS;
    printRow(slp::stepName(p), cfg, scs);
  }

  printHeader("B. controller steps, planner fixed to P2, plan once", scs);
  for (int c = lf::GOAL_SEEK; c <= lf::LIMITS; ++c) {
    cfg.planner_step = slp::ORIENTED;
    cfg.controller_step = c;
    printRow(lf::stepName(c), cfg, scs);
  }

  printHeader("C. planner x controller cross-check, plan once", scs);
  const int pairs[][2] = {{0, 0}, {0, 1}, {0, 2}, {1, 1}, {2, 1}, {1, 2}, {1, 4}};
  for (const auto & pr : pairs) {
    cfg.planner_step = pr[0];
    cfg.controller_step = pr[1];
    printRow(pc(pr[0], pr[1]), cfg, scs);
  }

  printDetailHeader("D. C4 ramps up and slows down near the goal (S1, P2, plan once)");
  for (int c : {lf::ROTATE, lf::LIMITS}) {
    cfg.planner_step = slp::ORIENTED;
    cfg.controller_step = c;
    printDetail(lf::stepName(c), scs[0], cfg);
  }

  printDetailHeader("E. replanning hides controller problems");
  struct Case
  {
    int p;
    int c;
    size_t scenario;
  };
  for (const Case & k : {Case{2, 2, 3}, Case{1, 1, 4}, Case{2, 4, 4}}) {
    for (bool replan : {false, true}) {
      cfg.planner_step = k.p;
      cfg.controller_step = k.c;
      cfg.replan = replan;
      printDetail(
        pc(k.p, k.c) + " " + scs[k.scenario].name.substr(0, 2) +
        (replan ? " / replan 1Hz" : " / plan once"), scs[k.scenario], cfg);
    }
  }
  cfg.replan = false;
  std::printf(
    "\nmaxCTE/rmsCTE: distance from the ORIGINAL line (start -> goal) [m]\n"
    "maxCTE*: distance from the LATEST received path [m]\n"
    "peak_acc/peak_dec: largest increase/decrease of the commanded linear velocity [m/s^2]\n");
}

int usage(const char * argv0)
{
  std::printf(
    "usage: %s [-s SCENARIO -p PLANNER_STEP -c CONTROLLER_STEP [--replan] [--trace FILE]]\n",
    argv0);
  return 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string scenario;
  std::string trace;
  SimConfig cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char * {return i + 1 < argc ? argv[++i] : nullptr;};
    if (a == "-s") {
      const char * v = next();
      if (!v) {return usage(argv[0]);}
      scenario = v;
    } else if (a == "-p") {
      const char * v = next();
      if (!v) {return usage(argv[0]);}
      cfg.planner_step = std::atoi(v);
    } else if (a == "-c") {
      const char * v = next();
      if (!v) {return usage(argv[0]);}
      cfg.controller_step = std::atoi(v);
    } else if (a == "--replan") {
      cfg.replan = true;
    } else if (a == "--trace") {
      const char * v = next();
      if (!v) {return usage(argv[0]);}
      trace = v;
    } else if (a == "--ros-args") {
      break;   // ros2 run 이 붙이는 인자는 무시
    } else {
      return usage(argv[0]);
    }
  }

  if (scenario.empty()) {
    tutorialTables();
    return 0;
  }

  for (const auto & sc : defaultScenarios()) {
    if (sc.name != scenario) {
      continue;
    }
    printDetailHeader(sc.name + ": " + sc.description);
    printDetail(
      pc(cfg.planner_step, cfg.controller_step) + (cfg.replan ? " / replan" : " / once"), sc,
      cfg);
    if (!trace.empty()) {
      const auto r = simulate(sc, cfg);
      std::ofstream f(trace);
      f << "t,x,y,yaw,v_cmd,w_cmd,state\n";
      for (const auto & s : r.trace) {
        f << s.t << ',' << s.pose.x << ',' << s.pose.y << ',' << s.pose.yaw << ',' << s.v <<
          ',' << s.w << ',' << s.state << '\n';
      }
      std::printf("trace written to %s (%zu samples)\n", trace.c_str(), r.trace.size());
    }
    return 0;
  }
  std::printf("unknown scenario '%s'\n", scenario.c_str());
  return 1;
}
