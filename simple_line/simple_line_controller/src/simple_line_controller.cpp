#include "simple_line_controller/simple_line_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace simple_line_controller
{

namespace
{

double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quatOf(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

double sign(double v)
{
  return v >= 0.0 ? 1.0 : -1.0;
}

}  // namespace

void SimpleLineController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("SimpleLineController: unable to lock parent node");
  }
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();

  auto param = [&](const std::string & key, double & value) {
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + key, rclcpp::ParameterValue(value));
      node->get_parameter(plugin_name_ + "." + key, value);
    };
  param("desired_linear_vel", desired_linear_vel_);
  param("max_angular_vel", max_angular_vel_);
  param("lookahead_dist", lookahead_dist_);
  param("rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  param("rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  param("rotate_to_heading_tolerance", rotate_to_heading_tolerance_);
  param("rotate_settled_angular_vel", rotate_settled_angular_vel_);
  param("max_linear_accel", max_linear_accel_);
  param("max_linear_decel", max_linear_decel_);
  param("max_linear_jerk", max_linear_jerk_);
  param("max_angular_accel", max_angular_accel_);
  param("min_approach_linear_vel", min_approach_linear_vel_);
  param("obstacle_check_width", obstacle_check_width_);
  param("obstacle_stop_distance", obstacle_stop_distance_);
  param("obstacle_resume_delay", obstacle_resume_delay_);
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_check", rclcpp::ParameterValue(obstacle_check_));
  node->get_parameter(plugin_name_ + ".obstacle_check", obstacle_check_);
  param("transform_tolerance", transform_tolerance_);
  param("max_robot_pose_search_dist", max_robot_pose_search_dist_);

  double controller_frequency = 20.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_duration_ = 1.0 / controller_frequency;
  clock_ = node->get_clock();

  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("simple_line/local_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("simple_line/carrot", 1);

  RCLCPP_INFO(logger_, "Configured %s (SimpleLineController)", plugin_name_.c_str());
}

void SimpleLineController::cleanup()
{
  local_plan_pub_.reset();
  carrot_pub_.reset();
}

void SimpleLineController::activate()
{
  local_plan_pub_->on_activate();
  carrot_pub_->on_activate();
}

void SimpleLineController::deactivate()
{
  local_plan_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
}

void SimpleLineController::setPlan(const nav_msgs::msg::Path & path)
{
  plan_frame_ = path.header.frame_id;
  plan_.clear();
  plan_.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    plan_.push_back({ps.pose.position.x, ps.pose.position.y, yawOf(ps.pose.orientation)});
  }
}

std::vector<SimpleLineController::Pose2D> SimpleLineController::localPlan(const Pose2D & robot)
{
  const auto * costmap = costmap_ros_->getCostmap();
  const double max_dist =
    std::max(costmap->getSizeInMetersX(), costmap->getSizeInMetersY()) / 2.0;

  // 1) 로봇에 가장 가까운 점 앞까지 잘라낸다 (path pruning)
  //    경로 앞쪽에서 search_dist 만큼만 찾는다. 경로가 꺾여서 자기 자신과 교차하거나
  //    되돌아오는 경우(여러 goal 경유), 경로 전체에서 찾으면 나중 구간의 점을 잡아
  //    중간 구간을 건너뛴다.
  const double search_dist =
    max_robot_pose_search_dist_ > 0.0 ? max_robot_pose_search_dist_ : max_dist;
  size_t closest = 0;
  double best = std::numeric_limits<double>::max();
  double travelled = 0.0;
  for (size_t i = 0; i < plan_.size(); ++i) {
    if (i > 0) {
      travelled += std::hypot(plan_[i].x - plan_[i - 1].x, plan_[i].y - plan_[i - 1].y);
      if (travelled > search_dist) {
        break;
      }
    }
    const double d = std::hypot(plan_[i].x - robot.x, plan_[i].y - robot.y);
    if (d < best) {
      best = d;
      closest = i;
    }
  }
  plan_.erase(plan_.begin(), plan_.begin() + static_cast<long>(closest));

  // 2) local costmap 범위 안의 연속 구간만 로봇 좌표계로 옮긴다
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);

  std::vector<Pose2D> local;
  for (const auto & p : plan_) {
    const double dx = p.x - robot.x;
    const double dy = p.y - robot.y;
    if (std::hypot(dx, dy) > max_dist) {
      break;
    }
    local.push_back({c * dx + s * dy, -s * dx + c * dy, normalizeAngle(p.yaw - robot.yaw)});
  }
  return local;
}

double SimpleLineController::obstacleDistanceOnPath(
  const std::vector<Pose2D> & local, const geometry_msgs::msg::PoseStamped & pose) const
{
  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  const double yaw = yawOf(pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const int r = static_cast<int>(std::ceil(obstacle_check_width_ / costmap->getResolution()));
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());

  // 경로 점(planner 간격, 0.1 m) 사이도 costmap 칸 간격으로 본다. 경로 점에서만 보면 경로를 다시 계산할 때마다
  // 점의 위치가 바뀌어 거리가 0.1 m 씩 튄다. 서서 기다리다 길이 빈 것으로 보고 움찔 다시 가는 일이 생겼다
  const double step = costmap->getResolution();
  double travelled = 0.0;
  Pose2D prev;                        // 로봇 (로봇 좌표계의 원점)
  for (const auto & p : local) {
    const double seg = std::hypot(p.x - prev.x, p.y - prev.y);
    const int n = std::max(1, static_cast<int>(std::ceil(seg / step)));
    for (int k = 1; k <= n; ++k) {
      const double f = static_cast<double>(k) / n;
      const double x = prev.x + f * (p.x - prev.x);
      const double y = prev.y + f * (p.y - prev.y);
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap->worldToMap(
          pose.pose.position.x + c * x - s * y, pose.pose.position.y + s * x + c * y, mx, my))
      {
        return std::numeric_limits<double>::infinity();   // local costmap 밖은 모른다
      }
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          const int cx = static_cast<int>(mx) + dx;
          const int cy = static_cast<int>(my) + dy;
          if (dx * dx + dy * dy > r * r || cx < 0 || cy < 0 || cx >= size_x || cy >= size_y) {
            continue;
          }
          if (costmap->getCost(cx, cy) != nav2_costmap_2d::LETHAL_OBSTACLE) {
            continue;
          }
          // 로봇(회전 중심)보다 뒤에 있는 것은 가는 길 위가 아니다. 경로의 첫 점은 로봇 자리라서, 그 둘레를 보면
          // 돌아서서 멀어지려는 장애물까지 막힌 것으로 본다
          double wx = 0.0;
          double wy = 0.0;
          costmap->mapToWorld(cx, cy, wx, wy);
          if (c * (wx - pose.pose.position.x) + s * (wy - pose.pose.position.y) <= 0.0) {
            continue;
          }
          return travelled + f * seg;
        }
      }
    }
    travelled += seg;
    prev = p;
  }
  return std::numeric_limits<double>::infinity();
}

geometry_msgs::msg::TwistStamped SimpleLineController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (plan_.empty()) {
    throw nav2_core::PlannerException("SimpleLineController: received plan with zero length");
  }

  // 로봇 pose(odom)를 경로 frame(map)으로
  geometry_msgs::msg::PoseStamped robot_ps;
  if (!transformPose(plan_frame_, pose, robot_ps)) {
    throw nav2_core::PlannerException("SimpleLineController: unable to transform robot pose");
  }
  const Pose2D robot{
    robot_ps.pose.position.x, robot_ps.pose.position.y, yawOf(robot_ps.pose.orientation)};

  const auto local = localPlan(robot);
  publishLocalPlan(local, pose.header.stamp);
  if (local.empty()) {
    throw nav2_core::PlannerException("SimpleLineController: no path inside the local costmap");
  }

  // goal = 경로의 마지막 점.
  // goal 까지 거리는 직선 거리가 아니라 '남은 경로 길이'로 잰다. 여러 goal 을 경유하는
  // 꺾은선 경로에서는 최종 goal 이 가까이 보여도 아직 돌아야 할 구간이 남아 있을 수 있다.
  const Pose2D & g = plan_.back();
  double goal_dist = std::hypot(plan_.front().x - robot.x, plan_.front().y - robot.y);
  for (size_t i = 1; i < plan_.size(); ++i) {
    goal_dist += std::hypot(plan_[i].x - plan_[i - 1].x, plan_[i].y - plan_[i - 1].y);
  }
  const double goal_yaw_err = normalizeAngle(g.yaw - robot.yaw);

  // goal checker 의 xy 허용오차 = '도착했다'고 보는 거리
  double goal_tolerance = 0.25;
  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tol;
    geometry_msgs::msg::Twist vel_tol;
    if (goal_checker->getTolerances(pose_tol, vel_tol)) {
      goal_tolerance = pose_tol.position.x;
    }
  }

  // 속도 제한 (speed filter 등)
  double v_max = desired_linear_vel_;
  if (speed_limit_ != nav2_costmap_2d::NO_SPEED_LIMIT) {
    v_max = std::min(
      v_max, speed_limit_is_percentage_ ? desired_linear_vel_ * speed_limit_ / 100.0 :
      speed_limit_);
  }

  // carrot: lookahead_dist 이상 떨어진 첫 점, 없으면 마지막 점
  auto it = std::find_if(
    local.begin(), local.end(), [&](const Pose2D & p) {
      return std::hypot(p.x, p.y) >= lookahead_dist_;
    });
  const Pose2D carrot = (it != local.end()) ? *it : local.back();
  const double carrot_angle = std::atan2(carrot.y, carrot.x);

  const double dt = control_duration_;
  // 가감속은 바로 전 주기에 낸 명령에서 이어 간다. 명령이 끊겼다가 다시 시작하면(새 goal)
  // 로봇의 실제 속도에서 시작한다
  const rclcpp::Time now = clock_->now();
  const bool continuing = has_last_cmd_ && (now - last_cmd_time_).seconds() < 3.0 * dt;
  Motion m = continuing ? motion_ : Motion{velocity.linear.x, 0.0};
  const double w0 = continuing ? last_w_ : velocity.angular.z;
  const MotionLimits lin{max_linear_accel_, max_linear_decel_, max_linear_jerk_};

  // 명령을 내는데 로봇이 1초 넘게 서 있으면(collision_monitor 의 StopZone, 비상정지, 무언가에 걸림)
  // 명령을 저속(min_approach_linear_vel)으로 낮춰 둔다. 막힌 것이 풀렸을 때 순항 속도로 튀어 나가지
  // 않고 저속에서 S 커브로 다시 가속한다
  const double creep = std::min(min_approach_linear_vel_, desired_linear_vel_);
  if (std::fabs(velocity.linear.x) >= 0.02) {
    held_ = false;
  } else if (continuing && m.v > creep + 0.01 && !held_) {
    held_ = true;
    held_since_ = now;
  }
  const bool held_long = held_ && (now - held_since_).seconds() > 1.0;
  if (held_long && m.v > creep) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 5000, "robot is not moving with %.2f m/s commanded, holding at %.2f m/s",
      m.v, creep);
    m = Motion{creep, 0.0};
  }

  auto limit_w = [&](double w) {
      return std::clamp(w, w0 - max_angular_accel_ * dt, w0 + max_angular_accel_ * dt);
    };
  // 제자리 회전 속도. 남은 각도 err 에서 감속도 a 로 멈출 수 있는 속도 sqrt(2·a·|err|) 를 넘지 않게 해서,
  // 목표 방향을 지나치지 않고 감속하며 멈춘다. a 는 max_angular_accel 의 절반으로 잡는다. 명령이
  // velocity_smoother 를 거쳐 로봇에 닿기까지 지연이 있어서, 한계값 그대로 계획하면 3~6° 지나쳤다
  auto rotate_w = [&](double err) {
      const double speed = std::min(
        rotate_to_heading_angular_vel_, std::sqrt(max_angular_accel_ * std::fabs(err)));
      return limit_w(sign(err) * speed);
    };

  // 제자리 회전을 시작하는 경우:
  //   - 주행 중에 경로가 rotate_to_heading_min_angle 보다 크게 틀어졌을 때 (꺾인 경로)
  //   - 서 있는 상태에서 출발할 때 방향이 rotate_to_heading_tolerance 보다 틀어져 있을 때
  // 한 번 시작하면 방향 오차가 rotate_to_heading_tolerance 안으로 들어오고 회전이 멈출 때까지 계속한다.
  // 덜 돈 채로 출발하면 남은 오차를 pure pursuit 가 호를 그리며 줄여서 경로가 휜다.
  // 장애물 앞에서 기다리는 동안에는 출발 전 방향 맞추기(작은 회전)를 하지 않는다. 회전 명령이 막혀 있으면
  // 오차와 함께 커지기만 하다가 풀리는 순간 한 번에 나간다. 경로가 크게 틀어졌으면(새 goal 이 반대쪽)
  // 기다리지 않고 바로 돈다. 장애물 상태는 지난 방향의 것이므로 비우고, 돈 뒤 새 경로에서 다시 본다
  const bool stopped = std::fabs(velocity.linear.x) < 0.01;
  const bool turn = std::fabs(carrot_angle) > rotate_to_heading_min_angle_;
  if (!rotating_ &&
    (turn || (stopped && !obstacle_braking_ && std::fabs(carrot_angle) > rotate_to_heading_tolerance_)))
  {
    rotating_ = true;
    obstacle_braking_ = false;
    clear_since_.reset();
  }
  if (rotating_ && std::fabs(carrot_angle) <= rotate_to_heading_tolerance_ &&
    std::fabs(velocity.angular.z) <= rotate_settled_angular_vel_)
  {
    rotating_ = false;
  }

  double w = 0.0;
  if (goal_dist <= goal_tolerance) {
    // goal 의 xy 에 도착: 서서 goal yaw 로 회전
    rotating_ = false;
    braking_ = false;
    m = stepToward(m, 0.0, lin, dt);
    w = rotate_w(goal_yaw_err);
  } else if (rotating_) {
    // 경로 쪽으로 제자리 회전. 달리던 중이면 먼저 곧게 서고, 방향이 맞았으면 회전이 멈출 때까지 0 을 낸다
    m = stepToward(m, 0.0, lin, dt);
    if (m.v <= 0.01) {
      w = std::fabs(carrot_angle) > rotate_to_heading_tolerance_ ? rotate_w(carrot_angle) : 0.0;
    }
  } else {
    // pure pursuit
    const double L2 = carrot.x * carrot.x + carrot.y * carrot.y;
    const double curvature = L2 > 1e-6 ? 2.0 * carrot.y / L2 : 0.0;

    // goal 앞 감속: 지금 브레이크를 걸면 min_approach_linear_vel 까지 줄이는 데 가는 거리가
    // 도착 범위까지 남은 거리에 닿으면 감속을 시작한다. 그 뒤 도착 범위까지 그 속도로 다가간다.
    // (경로가 바뀌어 남은 거리가 넉넉해지면 다시 가속)
    const double v_end = std::min(min_approach_linear_vel_, v_max);
    const double to_stop = goal_dist - goal_tolerance - m.v * dt;
    const double need = brakingDistance(m, v_end, lin, dt);
    if (braking_ && to_stop > need + 0.1) {
      braking_ = false;
    }
    if (need >= to_stop) {
      braking_ = true;
    }
    double target = braking_ ? v_end : v_max;

    // 경로 위 장애물: goal 과 같은 방식으로, 막힌 점 obstacle_stop_distance 앞에 0 으로 서도록 감속한다.
    // 장애물이 사라지면(local costmap 에서 지워지면) 다시 S 커브로 가속한다. 서서 기다리던 중이면
    // 길이 obstacle_resume_delay 동안 계속 비어 있어야 출발한다 (안전 스캐너의 재시작 지연과 같은 생각)
    if (obstacle_check_) {
      const double obstacle = obstacleDistanceOnPath(local, pose);
      const double to_obstacle = obstacle - obstacle_stop_distance_ - m.v * dt;
      const double need_stop = brakingDistance(m, 0.0, lin, dt);
      const bool was_braking = obstacle_braking_;
      // 감속을 푸는 것은 장애물이 0.3 m 넘게 멀어졌거나 사라졌을 때. costmap 칸이 한두 칸(5~10 cm) 흔들리거나
      // 필요보다 조금 일찍 감속을 시작해서 생긴 여유로는 풀지 않는다 (풀었다가 다시 감속하는 꿀렁임이 생겼다)
      const bool clear = to_obstacle > need_stop + 0.3;
      if (!clear) {
        clear_since_.reset();
      } else if (!clear_since_) {
        clear_since_ = now;
      }
      if (obstacle_braking_ && clear &&
        (m.v > 0.01 || (now - *clear_since_).seconds() >= obstacle_resume_delay_))
      {
        obstacle_braking_ = false;
      }
      if (need_stop >= to_obstacle) {
        obstacle_braking_ = true;
      }
      if (obstacle_braking_ != was_braking) {
        if (obstacle_braking_) {
          RCLCPP_INFO(logger_, "obstacle on the path %.2f m ahead, slowing down to stop", obstacle);
        } else {
          RCLCPP_INFO(logger_, "path clear, going again");
        }
      }
      if (obstacle_braking_) {
        target = 0.0;
      }
    }
    if (held_long) {
      target = std::min(target, creep);
    }
    m = stepToward(m, target, lin, dt);
    m.v = std::max(m.v, 0.0);

    w = m.v * curvature;
    // 각속도 상한에 걸리면 v 도 같이 줄여 곡률을 유지
    if (std::fabs(w) > max_angular_vel_) {
      m.v *= max_angular_vel_ / std::fabs(w);
      w = sign(w) * max_angular_vel_;
    }
  }
  const double v = m.v;
  motion_ = m;
  last_w_ = w;
  last_cmd_time_ = now;
  has_last_cmd_ = true;

  if (carrot_pub_->get_subscription_count() > 0) {
    auto msg = std::make_unique<geometry_msgs::msg::PointStamped>();
    msg->header.frame_id = costmap_ros_->getBaseFrameID();
    msg->header.stamp = pose.header.stamp;
    msg->point.x = carrot.x;
    msg->point.y = carrot.y;
    carrot_pub_->publish(std::move(msg));
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.twist.linear.x = v;
  cmd.twist.angular.z = w;
  return cmd;
}

void SimpleLineController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

bool SimpleLineController::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in,
  geometry_msgs::msg::PoseStamped & out) const
{
  if (in.header.frame_id == frame) {
    out = in;
    return true;
  }
  try {
    tf_->transform(in, out, frame, tf2::durationFromSec(transform_tolerance_));
    out.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

void SimpleLineController::publishLocalPlan(
  const std::vector<Pose2D> & local, const builtin_interfaces::msg::Time & stamp)
{
  if (local_plan_pub_->get_subscription_count() == 0) {
    return;
  }
  auto msg = std::make_unique<nav_msgs::msg::Path>();
  msg->header.frame_id = costmap_ros_->getBaseFrameID();
  msg->header.stamp = stamp;
  for (const auto & p : local) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg->header;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.orientation = quatOf(p.yaw);
    msg->poses.push_back(ps);
  }
  local_plan_pub_->publish(std::move(msg));
}

}  // namespace simple_line_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(simple_line_controller::SimpleLineController, nav2_core::Controller)
