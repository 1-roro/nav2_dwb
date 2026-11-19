// DOCKING_BIG_PICTURE.md 5장: "왜 칼만 필터가 필요한가" 를 실제 헤더(aruco_dock_control.hpp)로 비교한다.
//   python3 gen_meas.py                                        # meas.txt (합성 영상 108장을 실제 detector 로 잰 것)
//   g++ -std=c++17 -I../../include ex_why_filter.cpp -o ex_why_filter && ./ex_why_filter > why.csv
//   python3 gen_meas.py dock_b && ./ex_why_filter meas_dock_b.txt > why_b.csv     # marker 둘 (8-3)
//
// 같은 길(staging 0.75 m → 붙는 자리 0.31 m)을 지나는 동안, 네 가지 방법이 "로봇이 dock 에 대해 어디 있다고
// 생각하는지" 를 참값과 비교한다. 이 오차가 곧 제어가 0 으로 만들려는 오차라서, 끝에 남는 값이 도킹 오차가 된다.
//   raw        영상 한 장의 측정을 그대로 (필터 없음). 못 보면 마지막 측정을 들고 있는다
//   once       staging 에서 8장을 합쳐 한 번 잡고, 그 뒤로는 odom 만 믿는다
//   nopredict  칼만 필터지만 예측(불확실도 키우기)을 하지 않는다: 지금까지의 측정을 모두 똑같이 믿는다
//   filter     이 프로젝트의 방법 (ArucoServo 와 같은 순서: predict → predictMotion → 가까운 후보 → update)
//
// odom 은 시뮬레이션의 mecanum_slip_sim.py 와 같은 미끄러짐을 넣는다: 몸체는 명령의 앞 98 %, 옆 90 %, 회전 96 % 만
// 가고 옆으로 1 m 에 0.04 rad 돈다. odom 은 명령(바퀴가 돈 만큼)을 적분하므로 그만큼 틀린다.
// 앞 0.70 m 부근에서 1초(15장) 동안 marker 를 못 본다고 둔다 (반사, 흔들림).
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "bt_mission/aruco_dock_control.hpp"
namespace ad = bt_mission::aruco_dock;
using bt_mission::dock::Pose2;

static double deg(double r) {return r * 180.0 / M_PI;}

static ad::Mat3 rotate(const ad::Mat3 & c, double yaw)      // R C Rᵀ (x, y 만 돈다)
{
  const double co = std::cos(yaw), s = std::sin(yaw);
  const ad::Mat3 r{co, -s, 0, s, co, 0, 0, 0, 1};
  const ad::Mat3 rt{co, s, 0, -s, co, 0, 0, 0, 1};
  return ad::mul(ad::mul(r, c), rt);
}

struct Frame
{
  std::string phase;
  Pose2 truth;                 // 로봇의 참 자세 (창고)
  double dist;                 // D 에서 로봇의 x (설비 면에서 로봇 중심까지)
  std::vector<ad::Measurement> rel;   // 로봇(base)에서 본 dock 후보 (detector 의 출력과 같은 것)
};

int main(int argc, char ** argv)
{
  const Pose2 D{4.55, 1.0, M_PI};
  const std::string file = argc > 1 ? argv[1] : "meas.txt";
  std::ifstream in(file);
  if (!in) {
    std::fprintf(stderr, "%s not found: run gen_meas.py first\n", file.c_str());
    return 1;
  }
  std::vector<Frame> frames;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    Frame f;
    int nh;
    ss >> f.phase >> f.truth.x >> f.truth.y >> f.truth.yaw >> f.dist >> nh;
    for (int i = 0; i < nh; ++i) {
      ad::Measurement m;
      ss >> m.pose.x >> m.pose.y >> m.pose.yaw;
      for (double & v : m.cov) {ss >> v;}
      // meas.txt 는 창고(= 미끄러짐 없는 odom) 기준이다. 로봇에서 본 것으로 되돌린다
      ad::Measurement r;
      r.pose = ad::relative(f.truth, m.pose);
      r.cov = rotate(m.cov, -f.truth.yaw);
      f.rel.push_back(r);
    }
    frames.push_back(f);
  }

  // 미끄러지는 odom: 참 이동을 로봇 기준으로 나눠 "명령" 으로 되돌리고, 그것을 적분한다 (mecanum_slip_sim.py)
  const double sx = 0.98, sy = 0.90, sw = 0.96, yaw_per_lateral = 0.04;
  std::vector<Pose2> odom(frames.size());
  odom[0] = frames[0].truth;
  for (size_t k = 1; k < frames.size(); ++k) {
    const Pose2 d = ad::relative(frames[k - 1].truth, frames[k].truth);
    const double cy = d.y / sy;
    const Pose2 cmd{d.x / sx, cy, (d.yaw - yaw_per_lateral * cy * sy) / sw};
    odom[k] = ad::compose(odom[k - 1], cmd);
  }

  auto in_odom = [&](size_t k, const ad::Measurement & r) {
      ad::Measurement m;
      m.pose = ad::compose(odom[k], r.pose);       // TF odom <- base (찍은 시각) 로 옮긴다 (arucoMeasurements)
      m.cov = rotate(r.cov, odom[k].yaw);
      return m;
    };

  // staging 에서 8장 합치기 (DetectArucoDock)
  std::vector<ad::Hypotheses> look;
  size_t k = 0;
  for (; k < frames.size() && frames[k].phase == "look"; ++k) {
    ad::Hypotheses hs;
    for (const auto & r : frames[k].rel) {hs.push_back(in_odom(k, r));}
    look.push_back(hs);
  }
  int used = 0;
  const auto fused = ad::fuseDetections(look, 16.3, &used);
  if (!fused) {
    std::fprintf(stderr, "fuse failed\n");
    return 1;
  }
  Pose2 raw = fused->pose, once = fused->pose;
  ad::DockFilter filter, nopredict;
  filter.reset(fused->pose, fused->cov);
  nopredict.reset(fused->pose, fused->cov);

  std::printf("k,dist,phase,seen,raw_along,raw_lat,raw_yaw,once_along,once_lat,once_yaw,"
    "nopredict_along,nopredict_lat,nopredict_yaw,filter_along,filter_lat,filter_yaw,filter_sigma_yaw,"
    "odom_along,odom_lat,odom_yaw,meas_sigma_yaw,filter_sigma_along,filter_sigma_lat\n");
  Pose2 last_robot = odom[k - 1];
  double pending_dt = 0.0;
  int move_index = 0, rejected = 0;
  for (; k < frames.size(); ++k, ++move_index) {
    const Frame & f = frames[k];
    const bool seen = !(move_index >= 18 && move_index < 33) && !f.rel.empty();
    pending_dt += 1.0 / 15.0;
    double meas_sigma_yaw = 0.0;
    if (seen) {
      ad::Hypotheses hs;
      for (const auto & r : f.rel) {hs.push_back(in_odom(k, r));}
      raw = hs[0].pose;                              // 비용이 가장 작은 후보 하나를 그대로
      meas_sigma_yaw = std::sqrt(hs[0].cov[8]);
      // 이 프로젝트: 지난 측정 뒤로 흐른 시간과 움직인 만큼 불확실도를 키우고, 필터에 가까운 후보로 갱신
      filter.predict(pending_dt, 0.002, 0.002);
      filter.predictMotion(std::hypot(odom[k].x - last_robot.x, odom[k].y - last_robot.y),
        ad::normalizeAngle(odom[k].yaw - last_robot.yaw), 0.10, 0.05, 0.05);
      last_robot = odom[k];
      pending_dt = 0.0;
      const auto & m = hs[ad::closestHypothesis(filter, hs)];
      if (filter.update(m.pose, m.cov, 16.3, 8) == ad::DockFilter::Result::Rejected) {++rejected;}
      const auto & m2 = hs[ad::closestHypothesis(nopredict, hs)];
      nopredict.update(m2.pose, m2.cov, 16.3, 8);
    }
    // 오차 = (방법이 생각하는 로봇의 D 자세) - (참 D 자세). 제어는 앞의 것을 목표로 몰아가므로, 끝에 이것이 남는다
    const Pose2 truth_in_D = ad::relative(D, f.truth);
    auto err = [&](const Pose2 & dock_est) {
        const Pose2 p = ad::relative(dock_est, odom[k]);
        return Pose2{(p.x - truth_in_D.x) * 1000.0, (p.y - truth_in_D.y) * 1000.0,
          deg(ad::normalizeAngle(p.yaw - truth_in_D.yaw))};
      };
    const Pose2 er = err(raw), eo = err(once), en = err(nopredict.mean()), ef = err(filter.mean());
    // 필터의 dock 불확실도가 "로봇이 D 에서 어디 있나" 의 불확실도로 얼마나 되나 (깔때기가 보는 값): J Σ Jᵀ, J 는 수치 미분
    double jac[3][3];
    for (int c = 0; c < 3; ++c) {
      Pose2 a = filter.mean(), b = filter.mean();
      double * pa = c == 0 ? &a.x : (c == 1 ? &a.y : &a.yaw);
      double * pb = c == 0 ? &b.x : (c == 1 ? &b.y : &b.yaw);
      *pa += 1e-6;
      *pb -= 1e-6;
      const Pose2 qa = ad::relative(a, odom[k]), qb = ad::relative(b, odom[k]);
      jac[0][c] = (qa.x - qb.x) / 2e-6;
      jac[1][c] = (qa.y - qb.y) / 2e-6;
      jac[2][c] = ad::normalizeAngle(qa.yaw - qb.yaw) / 2e-6;
    }
    double var_along = 0.0, var_lat = 0.0;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        var_along += jac[0][i] * filter.cov()[i * 3 + j] * jac[0][j];
        var_lat += jac[1][i] * filter.cov()[i * 3 + j] * jac[1][j];
      }
    }
    std::printf("%zu,%.4f,%s,%d,%.3f,%.3f,%.4f,%.3f,%.3f,%.4f,%.3f,%.3f,%.4f,%.3f,%.3f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%.3f,%.3f\n",
      k, f.dist, f.phase.c_str(), seen ? 1 : 0, er.x, er.y, er.yaw, eo.x, eo.y, eo.yaw, en.x, en.y, en.yaw,
      ef.x, ef.y, ef.yaw, deg(std::sqrt(filter.cov()[8])),
      (odom[k].x - f.truth.x) * 1000.0, (odom[k].y - f.truth.y) * 1000.0, deg(ad::normalizeAngle(odom[k].yaw - f.truth.yaw)),
      deg(meas_sigma_yaw), std::sqrt(var_along) * 1000.0, std::sqrt(var_lat) * 1000.0);
  }
  std::fprintf(stderr, "fused %d of %zu looks, filter rejected %d\n", used, look.size(), rejected);
  return 0;
}
