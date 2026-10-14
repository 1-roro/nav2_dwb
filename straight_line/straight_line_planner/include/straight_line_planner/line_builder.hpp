// 직선 경로 생성 로직 (ROS 의존성 없음).
//
// 플러그인(straight_line_planner.cpp)과 오프라인 시뮬레이터(line_tutorial_sim)가
// 같은 코드를 쓰도록 헤더 하나에 모아 두었다.
//
// 단계 (dwb_steps.py 의 STEPS 와 같은 역할, 파라미터 `<planner>.step` 으로 선택)
//   0 ENDPOINTS   [start, goal] 두 점만                    <- 컨트롤러가 경로를 local costmap
//                                                             범위로 잘라 쓰면 goal 이 잘려 나간다
//   1 INTERPOLATE 공식 튜토리얼과 동일한 보간               <- 중간 pose 의 orientation 이 전부 yaw=0
//   2 ORIENTED    보간 + 각 pose 의 yaw 를 진행 방향으로     <- 1 의 결함을 고친다
//   3 COLLISION   2 + costmap 충돌 검사 (플러그인에서 수행)  <- 막힌 직선은 경로를 거부한다

#ifndef STRAIGHT_LINE_PLANNER__LINE_BUILDER_HPP_
#define STRAIGHT_LINE_PLANNER__LINE_BUILDER_HPP_

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace straight_line_planner
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

enum Step : int
{
  ENDPOINTS = 0,
  INTERPOLATE = 1,
  ORIENTED = 2,
  COLLISION = 3,
};

inline const char * stepName(int step)
{
  switch (step) {
    case ENDPOINTS: return "P0_endpoints";
    case INTERPOLATE: return "P1_interpolate";
    case ORIENTED: return "P2_oriented";
    default: return "P3_collision";
  }
}

// start -> goal 직선 경로. 마지막 pose 는 항상 goal (goal 의 yaw 유지).
inline std::vector<Pose2D> buildLine(
  const Pose2D & start, const Pose2D & goal, double resolution, int step)
{
  std::vector<Pose2D> poses;
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double dist = std::hypot(dx, dy);

  if (step <= ENDPOINTS) {
    poses.push_back(start);
    poses.push_back(goal);
    return poses;
  }

  if (step == INTERPOLATE) {
    // 공식 튜토리얼(nav2_straightline_planner) 코드를 그대로 옮긴 것.
    //  - 개수를 int 로 잘라서 계산한다. dist < resolution 이면 0 이 되어 goal 한 점만 남는다.
    //    (이때 dx / n 은 0 나누기지만 루프가 돌지 않으므로 쓰이지 않는다)
    //  - 중간 pose 의 orientation 을 단위 쿼터니언(yaw = 0, 즉 map 의 +x 방향)으로 둔다.
    const int n = static_cast<int>(dist / resolution);
    for (int i = 0; i < n; ++i) {
      poses.push_back({start.x + dx / n * i, start.y + dy / n * i, 0.0});
    }
    poses.push_back(goal);
    return poses;
  }

  // ORIENTED 이상: 모든 중간 pose 가 진행 방향을 바라보게 한다.
  // 올림(ceil)으로 개수를 정해 간격이 resolution 을 넘지 않게 하고, start 를 항상 포함한다.
  const double heading = dist > 1e-9 ? std::atan2(dy, dx) : goal.yaw;
  const int n = std::max(1, static_cast<int>(std::ceil(dist / resolution)));
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / n;
    poses.push_back({start.x + dx * t, start.y + dy * t, heading});
  }
  poses.push_back(goal);
  return poses;
}

// 선분 a -> b 를 sample_step 간격으로 훑으며 is_blocked(x, y) 가 처음 true 가 되는
// 지점까지의 거리 [m] 를 반환한다. 막힌 곳이 없으면 -1.
inline double firstBlockedDistance(
  const Pose2D & a, const Pose2D & b, double sample_step,
  const std::function<bool(double, double)> & is_blocked)
{
  const double dist = std::hypot(b.x - a.x, b.y - a.y);
  const int n = std::max(1, static_cast<int>(std::ceil(dist / sample_step)));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    if (is_blocked(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) {
      return dist * t;
    }
  }
  return -1.0;
}

}  // namespace straight_line_planner

#endif  // STRAIGHT_LINE_PLANNER__LINE_BUILDER_HPP_
