// DOCKING_STUDY.md 1-6, 3장, 4장의 숫자: 실제 헤더(aruco_dock_control.hpp)의 필터, 깔때기, mecanumStep 을 그대로 부른다.
//   g++ -std=c++17 -I../../include ex_filter.cpp -o ex_filter
//   ./ex_filter step      깔때기 표, mecanumStep 의 예 (4-3, 4-4), 갱신 하나 (1-6)
//   ./ex_filter filter    gen_meas.py 가 만든 meas.txt 를 필터에 차례로 넣는다 (3-3, 3-6)
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include "bt_mission/aruco_dock_control.hpp"
namespace ad = bt_mission::aruco_dock;
using bt_mission::dock::Pose2;
double deg(double r) {return r * 180.0 / M_PI;}
int main(int argc, char ** argv)
{
  const std::string mode = argc > 1 ? argv[1] : "filter";
  if (mode == "filter") {
    std::ifstream in("meas.txt");
    std::string line;
    std::vector<ad::Hypotheses> look;
    ad::DockFilter f;
    Pose2 last_robot; bool have_robot = false;
    const Pose2 truth{4.55, 1.0, M_PI};
    int k = 0;
    while (std::getline(in, line)) {
      std::istringstream ss(line);
      std::string phase; double rx, ry, ryaw, dist; int nh;
      ss >> phase >> rx >> ry >> ryaw >> dist >> nh;
      ad::Hypotheses hs;
      for (int i = 0; i < nh; ++i) {
        ad::Measurement m; ss >> m.pose.x >> m.pose.y >> m.pose.yaw;
        for (double & v : m.cov) ss >> v;
        hs.push_back(m);
      }
      const Pose2 robot{rx, ry, ryaw};
      if (phase == "look") {
        look.push_back(hs);
        if (look.size() == 8) {
          int used = 0;
          auto fused = ad::fuseDetections(look, 16.3, &used);
          f.reset(fused->pose, fused->cov);
          std::printf("FUSED used %d: x %.4f y %.4f yaw %.3f | sx %.2f sy %.2f mm syaw %.3f deg | err x %+.2f y %+.2f mm yaw %+.3f deg\n",
            used, fused->pose.x, fused->pose.y, deg(fused->pose.yaw), std::sqrt(fused->cov[0]) * 1000, std::sqrt(fused->cov[4]) * 1000,
            deg(std::sqrt(fused->cov[8])), (fused->pose.x - truth.x) * 1000, (fused->pose.y - truth.y) * 1000,
            deg(ad::normalizeAngle(fused->pose.yaw - truth.yaw)));
          for (size_t i = 0; i < look.size(); ++i)
            std::printf("  look %zu: z x %.4f y %.4f yaw %.3f sx %.2f syaw %.3f\n", i, look[i][0].pose.x, look[i][0].pose.y,
              deg(look[i][0].pose.yaw), std::sqrt(look[i][0].cov[0]) * 1000, deg(std::sqrt(look[i][0].cov[8])));
          last_robot = robot; have_robot = true;
        }
        continue;
      }
      // move: 예측 (1/15 s, 움직인 거리) -> 갱신
      f.predict(1.0 / 15.0, 0.002, 0.002);
      if (have_robot) f.predictMotion(std::hypot(robot.x - last_robot.x, robot.y - last_robot.y),
        ad::normalizeAngle(robot.yaw - last_robot.yaw), 0.10, 0.05, 0.05);
      const double before_yaw = std::sqrt(f.cov()[8]);
      last_robot = robot;
      const auto & m = hs[ad::closestHypothesis(f, hs)];
      const double d2 = ad::mahalanobis2(f.mean(), ad::add(f.cov(), m.cov), m.pose);
      const auto r = f.update(m.pose, m.cov, 16.3, 8);
      // dock 좌표계에서 로봇의 옆 오차가 필터 때문에 얼마나 틀렸나
      const Pose2 rel_est = ad::relative(f.mean(), robot), rel_true = ad::relative(truth, robot);
      std::printf("%3d d %.3f | meas sx %.2f mm syaw %.3f deg | d2 %.2f %s | filt syaw before %.3f after %.3f deg, sx %.2f mm | err lat %+.2f mm along %+.2f mm yaw %+.3f deg\n",
        ++k, dist, std::sqrt(m.cov[0]) * 1000, deg(std::sqrt(m.cov[8])), d2,
        r == ad::DockFilter::Result::Accepted ? "acc" : (r == ad::DockFilter::Result::Rejected ? "REJ" : "REINIT"),
        deg(before_yaw), deg(std::sqrt(f.cov()[8])), std::sqrt(f.cov()[0]) * 1000,
        (rel_est.y - rel_true.y) * 1000, (rel_est.x - rel_true.x) * 1000, deg(ad::normalizeAngle(rel_est.yaw - rel_true.yaw)));
    }
    return 0;
  }
  if (mode == "step") {
    // insert 단계의 값 (docking.yaml)
    ad::MecanumParams p;
    p.max_vx = 0.05; p.max_vy = 0.03; p.accel_x = 0.1; p.accel_y = 0.1;
    p.tol_along = 0.003; p.tol_lat = 0.002; p.tol_yaw = 0.3 * M_PI / 180;
    p.funnel_lat0 = 0.004; p.funnel_lat_slope = 0.15; p.funnel_yaw0 = 0.6 * M_PI / 180; p.funnel_yaw_slope = 14.0 * M_PI / 180;
    struct Case {double along, lat, yawdeg, advance;} cases[] = {{0.20, 0.012, 1.5, 1}, {0.20, 0.030, 1.5, 1}, {0.20, 0.012, 1.5, 0}, {0.004, 0.0, 0.0, 1}, {0.0, 0.0025, 0.0, 1}};
    for (auto c : cases) {
      ad::DockErrors e{c.along, c.lat, c.yawdeg * M_PI / 180};
      const double robot_yaw_in_dock = M_PI + e.yaw;
      const double d = std::max(e.along, 0.0);
      const double band_lat = p.funnel_lat0 + p.funnel_lat_slope * d, band_yaw = p.funnel_yaw0 + p.funnel_yaw_slope * d;
      const double g = ad::funnelGate(e, p);
      ad::Cmd last{0.04, 0.0, 0.0};
      const auto cmd = ad::mecanumStep(e, robot_yaw_in_dock, last, p, 0.05, c.advance);
      const auto cmd_free = ad::mecanumStep(e, robot_yaw_in_dock, last, p, 10.0, c.advance);   // 가속 제한 없이 원한 값
      std::printf("e along %.1f mm lat %.1f mm yaw %.2f deg adv %.0f | band lat %.1f mm (gate_lat %.2f) band yaw %.2f deg (gate_yaw %.2f) gate %.2f"
        " | want vx %+.4f vy %+.4f wz %+.4f | cmd (last vx 0.04, dt 0.05) vx %+.4f vy %+.4f wz %+.4f\n",
        e.along * 1000, e.lat * 1000, c.yawdeg, c.advance, band_lat * 1000, ad::funnelGate(e.lat, band_lat), deg(band_yaw),
        ad::funnelGate(e.yaw, band_yaw), g, cmd_free.vx, cmd_free.vy, cmd_free.wz, cmd.vx, cmd.vy, cmd.wz);
    }
    for (double dd : {0.0, 0.05, 0.10, 0.25, 0.45})
      std::printf("funnel d %.2f m: lat band %.1f mm (full speed inside %.1f mm), yaw band %.2f deg\n", dd,
        (p.funnel_lat0 + p.funnel_lat_slope * dd) * 1000, 0.5 * (p.funnel_lat0 + p.funnel_lat_slope * dd) * 1000,
        deg(p.funnel_yaw0 + p.funnel_yaw_slope * dd));
    // 갱신 하나를 손으로: 1차원 예
    ad::DockFilter f; f.reset({4.553, 1.0, M_PI}, ad::diag3(4e-6, 1e-6, std::pow(2 * M_PI / 180, 2)));
    const auto r = f.update({4.551, 1.0, M_PI - 0.5 * M_PI / 180}, ad::diag3(4e-6, 1e-6, std::pow(1 * M_PI / 180, 2)), 16.3, 8);
    std::printf("update example: x %.4f yaw %.3f deg, sx %.2f mm syaw %.3f deg (%d)\n", f.mean().x, deg(f.mean().yaw),
      std::sqrt(f.cov()[0]) * 1000, deg(std::sqrt(f.cov()[8])), (int)r);
    return 0;
  }
}
