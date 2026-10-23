#include "bt_fixed_mission/pose_nodes.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace bt_fixed_mission
{

namespace
{

std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\n\r");
  return s.substr(begin, end - begin + 1);
}

std::vector<std::string> split(const std::string & s, char delim)
{
  std::vector<std::string> out;
  std::string::size_type start = 0;
  while (true) {
    const auto pos = s.find(delim, start);
    out.push_back(s.substr(start, pos == std::string::npos ? std::string::npos : pos - start));
    if (pos == std::string::npos) {
      return out;
    }
    start = pos + 1;
  }
}

double toDouble(const std::string & text)
{
  const std::string s = trim(text);
  std::size_t used = 0;
  double value = 0.0;
  try {
    value = std::stod(s, &used);
  } catch (const std::exception &) {
    used = 0;
  }
  if (s.empty() || used != s.size()) {
    throw std::invalid_argument("'" + text + "' is not a number");
  }
  return value;
}

}  // namespace

std::vector<Pose2D> parsePoses(const std::string & text)
{
  std::vector<Pose2D> poses;
  std::vector<bool> has_yaw;
  for (const auto & item : split(text, ';')) {
    if (trim(item).empty()) {
      continue;
    }
    const auto fields = split(item, ',');
    if (fields.size() != 2 && fields.size() != 3) {
      throw std::invalid_argument("'" + trim(item) + "' is not 'x,y' or 'x,y,yaw_deg'");
    }
    Pose2D p;
    p.x = toDouble(fields[0]);
    p.y = toDouble(fields[1]);
    if (fields.size() == 3) {
      p.yaw = toDouble(fields[2]) * M_PI / 180.0;
    }
    poses.push_back(p);
    has_yaw.push_back(fields.size() == 3);
  }
  if (poses.empty()) {
    throw std::invalid_argument("no pose in '" + text + "'");
  }

  for (std::size_t i = 0; i < poses.size(); ++i) {
    if (has_yaw[i]) {
      continue;
    }
    if (i + 1 < poses.size()) {          // 중간 점: 다음 점 쪽
      poses[i].yaw = std::atan2(poses[i + 1].y - poses[i].y, poses[i + 1].x - poses[i].x);
    } else if (i > 0) {                  // 마지막 점: 이전 점에서 온 방향
      poses[i].yaw = std::atan2(poses[i].y - poses[i - 1].y, poses[i].x - poses[i - 1].x);
    } else {
      throw std::invalid_argument("a single pose needs a yaw: '" + trim(text) + "'");
    }
  }
  return poses;
}

geometry_msgs::msg::PoseStamped toPoseStamped(
  const Pose2D & pose, const std::string & frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = frame;
  out.header.stamp = stamp;
  out.pose.position.x = pose.x;
  out.pose.position.y = pose.y;
  out.pose.orientation.z = std::sin(pose.yaw / 2.0);
  out.pose.orientation.w = std::cos(pose.yaw / 2.0);
  return out;
}

// ---------------------------------------------------------------------------------------------

MakePose::MakePose(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
{
}

BT::NodeStatus MakePose::tick()
{
  std::string text, frame;
  getInput("pose", text);
  getInput("frame", frame);
  try {
    const auto poses = parsePoses(text);
    if (poses.size() != 1) {
      throw std::invalid_argument("expected one pose, got " + std::to_string(poses.size()));
    }
    setOutput("output", toPoseStamped(poses.front(), frame, node_->now()));
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(node_->get_logger(), "%s: %s", name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------

MakePoses::MakePoses(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
{
}

BT::NodeStatus MakePoses::tick()
{
  std::string text, frame;
  getInput("poses", text);
  getInput("frame", frame);
  std::vector<geometry_msgs::msg::PoseStamped> out;
  try {
    const auto stamp = node_->now();
    for (const auto & pose : parsePoses(text)) {
      out.push_back(toPoseStamped(pose, frame, stamp));
    }
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(node_->get_logger(), "%s: %s", name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }
  setOutput("output", out);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------------------------

void registerPoseNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<MakePose>("MakePose");
  factory.registerNodeType<MakePoses>("MakePoses");
}

}  // namespace bt_fixed_mission

// mission_bt_runner 가 extra_plugin_libs 로 이 라이브러리를 불러올 때 등록된다
BT_REGISTER_NODES(factory)
{
  bt_fixed_mission::registerPoseNodes(factory);
}
