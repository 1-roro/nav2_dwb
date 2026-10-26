#ifndef SIMPLE_LINE_CONTROLLER__MOTION_PROFILE_HPP_
#define SIMPLE_LINE_CONTROLLER__MOTION_PROFILE_HPP_

#include <algorithm>
#include <cmath>

namespace simple_line_controller
{

// 직선 속도의 가감속 프로파일 (S 커브).
//
// 속도를 목표값으로 바꿀 때 가속도도 한 번에 바꾸지 않고 jerk 로 제한해서 바꾼다.
// 그러면 속도-시간 그래프의 모서리가 둥글어진다 (가속 시작, 가속 끝, 감속 시작, 정지).
//
//   속도 ▲        ____________
//        │      /              \        jerk 제한 없음: 모서리가 꺾인 사다리꼴
//        │    /                  \      jerk 제한 있음: 모서리가 둥근 S 커브
//        └──────────────────────────▶ 시간
//
// 제어 주기마다 stepToward() 로 한 칸 나아간다. goal 앞에서는 brakingDistance() 로
// "지금 브레이크를 걸면 얼마나 가서 서는지" 를 같은 계산으로 미리 굴려 보고, 남은 거리와 같아지면 감속한다.

struct Motion
{
  double v{0.0};   // 속도 [m/s]
  double a{0.0};   // 가속도 [m/s^2]
};

struct MotionLimits
{
  double accel{0.5};   // 최대 가속도 [m/s^2]
  double decel{0.5};   // 최대 감속도 [m/s^2] (양수)
  double jerk{0.0};    // 최대 jerk [m/s^3]. 0 이하면 제한 없음 (가속도가 바로 바뀐다)
};

// 한 주기(dt) 동안 속도를 target 쪽으로 바꾼다. target 을 넘어서지 않는다.
inline Motion stepToward(const Motion & m, double target, const MotionLimits & lim, double dt)
{
  const double e = target - m.v;
  if (lim.jerk <= 0.0) {
    const double dv = std::clamp(e, -lim.decel * dt, lim.accel * dt);
    return {m.v + dv, dv / dt};
  }
  // target 에 닿는 순간 가속도도 0 이 되도록, 남은 속도 차이 e 에서 jerk 로 줄여 갈 수 있는 가속도까지만 쓴다.
  // 연속 시간이면 sqrt(2·j·|e|). 아래는 dt 간격으로 줄여 가는 경우의 값이다
  const double jdt = lim.jerk * dt;
  const double a_brake = std::sqrt(jdt * jdt / 4.0 + 2.0 * lim.jerk * std::fabs(e)) - jdt / 2.0;
  const double a_max = e > 0.0 ? lim.accel : lim.decel;
  const double a_want = e == 0.0 ? 0.0 : std::copysign(std::min(a_max, a_brake), e);
  Motion out;
  out.a = m.a + std::clamp(a_want - m.a, -jdt, jdt);
  out.v = m.v + out.a * dt;
  if (e == 0.0 || (e > 0.0 && out.v > target) || (e < 0.0 && out.v < target)) {
    out.v = target;
    out.a = std::clamp((target - m.v) / dt, m.a - jdt, m.a + jdt);
  }
  return out;
}

// m 에서 v_end 까지 stepToward 로 줄일 때 가는 거리 [m]. 이미 v_end 이하이고 가속 중이 아니면 0
inline double brakingDistance(Motion m, double v_end, const MotionLimits & lim, double dt)
{
  double s = 0.0;
  for (int i = 0; i < 10000 && (m.v > v_end + 1e-9 || m.a > 1e-9); ++i) {
    const double v0 = m.v;
    m = stepToward(m, v_end, lim, dt);
    s += 0.5 * (v0 + m.v) * dt;   // 한 주기 동안의 평균 속도
  }
  return s;
}

}  // namespace simple_line_controller

#endif  // SIMPLE_LINE_CONTROLLER__MOTION_PROFILE_HPP_
