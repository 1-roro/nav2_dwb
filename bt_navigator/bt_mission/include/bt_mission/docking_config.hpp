#ifndef BT_MISSION__DOCKING_CONFIG_HPP_
#define BT_MISSION__DOCKING_CONFIG_HPP_

#include <cmath>
#include <cstdlib>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "bt_mission/aruco_dock_control.hpp"
#include "yaml-cpp/yaml.h"

// config/docking.yaml 을 읽는다 (ROS 와 무관. test_docking_config.cpp 가 시험한다).
//
// 로봇, 카메라, 설비마다 달라지는 값은 모두 그 파일에 있고, 도킹 BT 노드는 XML 에서 단계 이름(stage)만 받는다.
// 값을 찾는 순서 (단계의 값): stages.<stage>.<이름> -> motion.<이름> -> 코드의 기본값.
// 실제 로봇으로 옮길 때는 그 파일만 고친다. 여기 기본값은 파일이 없거나 항목이 빠졌을 때만 쓰인다.

namespace bt_mission
{

class DockingConfig
{
public:
  struct Dock
  {
    std::string name;
    int marker_id{0};                 // marker 가 여럿이면 첫 번째
    std::vector<int> marker_ids;      // 이 dock 의 marker 전부 (markers: 목록, 없으면 marker_id 하나)
    double size{0.06};
    double height{0.13};
    double offset{0.0};
    double lateral{0.0};
    double gap{0.15};          // 붙었을 때 로봇 앞면 <-> 도킹 면
  };

  DockingConfig() = default;

  static DockingConfig fromFile(const std::string & path, std::string * error = nullptr)
  {
    DockingConfig c;
    c.path_ = path;
    try {
      c.root_ = YAML::LoadFile(path);
    } catch (const std::exception & e) {
      if (error) {
        *error = e.what();
      }
      c.root_ = YAML::Node();
    }
    return c;
  }

  static DockingConfig fromString(const std::string & text)
  {
    DockingConfig c;
    c.root_ = YAML::Load(text);
    return c;
  }

  bool loaded() const {return root_.IsMap();}
  const std::string & path() const {return path_;}

  // ---- 값 하나 ----
  double number(const std::string & section, const std::string & key, double fallback) const
  {
    const YAML::Node n = find(section, key);
    return n.IsScalar() ? n.as<double>(fallback) : fallback;
  }

  std::string text(const std::string & section, const std::string & key, const std::string & fallback) const
  {
    const YAML::Node n = find(section, key);
    if (n.IsNull()) {
      return n.IsDefined() ? std::string() : fallback;       // "key:" 뒤가 비었으면 빈 문자열
    }
    return n.IsScalar() ? n.as<std::string>(fallback) : fallback;
  }

  bool flag(const std::string & section, const std::string & key, bool fallback) const
  {
    const YAML::Node n = find(section, key);
    return n.IsScalar() ? n.as<bool>(fallback) : fallback;
  }

  std::vector<double> numbers(
    const std::string & section, const std::string & key, const std::vector<double> & fallback) const
  {
    const YAML::Node n = find(section, key);
    if (!n.IsSequence()) {
      return fallback;
    }
    std::vector<double> out;
    for (const auto & v : n) {
      out.push_back(v.as<double>());
    }
    return out;
  }

  // ---- 단계의 값: stages.<stage>.<key> -> motion.<key> -> fallback ----
  double stageNumber(const std::string & stage, const std::string & key, double fallback) const
  {
    const YAML::Node n = findStage(stage, key);
    return n.IsScalar() ? n.as<double>(fallback) : fallback;
  }

  bool stageFlag(const std::string & stage, const std::string & key, bool fallback) const
  {
    const YAML::Node n = findStage(stage, key);
    return n.IsScalar() ? n.as<bool>(fallback) : fallback;
  }

  aruco_dock::MecanumParams mecanum(const std::string & stage) const
  {
    aruco_dock::MecanumParams p;
    const double deg = M_PI / 180.0;
    p.k_along = stageNumber(stage, "k_along", p.k_along);
    p.k_lat = stageNumber(stage, "k_lat", p.k_lat);
    p.k_yaw = stageNumber(stage, "k_yaw", p.k_yaw);
    p.max_vx = stageNumber(stage, "max_vx", p.max_vx);
    p.max_vy = stageNumber(stage, "max_vy", p.max_vy);
    p.max_wz = stageNumber(stage, "max_wz", p.max_wz);
    p.min_vx = stageNumber(stage, "min_vx", p.min_vx);
    p.min_vy = stageNumber(stage, "min_vy", p.min_vy);
    p.min_wz = stageNumber(stage, "min_wz", p.min_wz);
    p.accel_x = stageNumber(stage, "accel_x", p.accel_x);
    p.accel_y = stageNumber(stage, "accel_y", p.accel_y);
    p.accel_wz = stageNumber(stage, "accel_wz", p.accel_wz);
    p.scale_vx = stageNumber(stage, "scale_vx", p.scale_vx);
    p.scale_vy = stageNumber(stage, "scale_vy", p.scale_vy);
    p.scale_wz = stageNumber(stage, "scale_wz", p.scale_wz);
    p.tol_along = stageNumber(stage, "tol_along", p.tol_along);
    p.tol_lat = stageNumber(stage, "tol_lat", p.tol_lat);
    p.tol_yaw = stageNumber(stage, "tol_yaw_deg", p.tol_yaw / deg) * deg;
    p.funnel_lat0 = stageNumber(stage, "funnel_lat0", p.funnel_lat0);
    p.funnel_lat_slope = stageNumber(stage, "funnel_lat_slope", p.funnel_lat_slope);
    p.funnel_yaw0 = stageNumber(stage, "funnel_yaw0_deg", p.funnel_yaw0 / deg) * deg;
    p.funnel_yaw_slope = stageNumber(stage, "funnel_yaw_slope_deg", p.funnel_yaw_slope / deg) * deg;
    return p;
  }

  // ---- dock ----
  std::vector<std::string> dockNames() const
  {
    std::vector<std::string> out;
    const YAML::Node docks = child(root_, "docks");
    if (docks.IsMap()) {
      for (const auto & kv : docks) {
        out.push_back(kv.first.as<std::string>());
      }
    }
    return out;
  }

  std::optional<Dock> dock(const std::string & name) const
  {
    const YAML::Node n = child(child(root_, "docks"), name);
    if (!n.IsMap()) {
      return std::nullopt;
    }
    Dock d;
    d.name = name;
    d.marker_id = child(n, "marker_id").as<int>(d.marker_id);
    const YAML::Node markers = child(n, "markers");
    if (markers.IsSequence()) {
      for (const auto & m : markers) {
        d.marker_ids.push_back(child(m, "id").as<int>(0));
      }
    }
    if (d.marker_ids.empty()) {
      d.marker_ids.push_back(d.marker_id);
    }
    d.marker_id = d.marker_ids.front();
    d.size = child(n, "size").as<double>(d.size);
    d.height = child(n, "height").as<double>(d.height);
    d.offset = child(n, "offset").as<double>(d.offset);
    d.lateral = child(n, "lateral").as<double>(d.lateral);
    d.gap = child(n, "gap").as<double>(d.gap);
    return d;
  }

  // 도면 값의 설 곳: 로봇 중심(base)이 도킹 면에서 떨어진 거리 = gap + 로봇 앞면까지
  double dockedDistance(const std::string & dock_name) const
  {
    const auto d = dock(dock_name);
    return (d ? d->gap : Dock().gap) + number("robot", "front", 0.16);
  }

  static std::string expandHome(const std::string & path)
  {
    const char * home = std::getenv("HOME");
    if (!path.empty() && path[0] == '~' && home) {
      return std::string(home) + path.substr(1);
    }
    return path;
  }

  // 파일에 있지만 코드가 모르는 이름 (오타). "motion.max_vxx" 같은 꼴
  std::vector<std::string> unknownKeys() const
  {
    static const std::set<std::string> motion{
      "max_vx", "max_vy", "max_wz", "min_vx", "min_vy", "min_wz", "accel_x", "accel_y", "accel_wz",
      "scale_vx", "scale_vy", "scale_wz", "k_along", "k_lat", "k_yaw", "tol_along", "tol_lat", "tol_yaw_deg",
      "funnel_lat0", "funnel_lat_slope", "funnel_yaw0_deg", "funnel_yaw_slope_deg",
      "odom_drift", "odom_slip", "odom_yaw_per_m", "odom_turn_slip"};
    static const std::set<std::string> stage_only{
      "standoff", "lost_timeout", "timeout", "settle", "hold_if_closer", "check_behind", "settle_time",
      "measure_time", "stable_spread", "max_measure_time", "max_iterations", "accept_along", "accept_lat",
      "accept_yaw_deg", "move_timeout", "all_markers"};
    static const std::map<std::string, std::set<std::string>> sections{
      {"robot", {"front", "rear", "half_width", "turn_radius"}},
      {"ros", {"odom_frame", "base_frame", "cmd_vel_topic", "scan_topic", "detection_topic"}},
      {"motion", motion},
      {"detect", {"settle_time", "look_time", "min_detections", "min_found", "search_angles_deg", "search_wz",
          "clearance", "timeout", "use_taught", "taught_dir"}},
      {"safety", {"fresh_detection", "min_border_px", "min_camera_distance"}},
      {"undock", {"distance", "speed", "timeout", "clearance"}},
    };
    static const std::set<std::string> dock_keys{"marker_id", "markers", "size", "height", "offset", "lateral", "gap"};
    static const std::set<std::string> marker_keys{"id", "size", "height", "offset", "lateral"};
    std::vector<std::string> out;
    if (!root_.IsMap()) {
      return out;
    }
    for (const auto & kv : root_) {
      const std::string section = kv.first.as<std::string>();
      if (section == "dictionary") {
        continue;
      }
      if (section == "docks") {
        for (const auto & dock : kv.second) {
          for (const auto & item : dock.second) {
            const std::string key = item.first.as<std::string>();
            if (!dock_keys.count(key)) {
              out.push_back("docks." + dock.first.as<std::string>() + "." + key);
            } else if (key == "markers" && item.second.IsSequence()) {
              for (const auto & marker : item.second) {
                for (const auto & value : marker) {
                  if (!marker_keys.count(value.first.as<std::string>())) {
                    out.push_back(
                      "docks." + dock.first.as<std::string>() + ".markers." + value.first.as<std::string>());
                  }
                }
              }
            }
          }
        }
      } else if (section == "stages") {
        for (const auto & stage : kv.second) {
          for (const auto & item : stage.second) {
            const std::string key = item.first.as<std::string>();
            if (!motion.count(key) && !stage_only.count(key)) {
              out.push_back("stages." + stage.first.as<std::string>() + "." + key);
            }
          }
        }
      } else if (sections.count(section)) {
        for (const auto & item : kv.second) {
          if (!sections.at(section).count(item.first.as<std::string>())) {
            out.push_back(section + "." + item.first.as<std::string>());
          }
        }
      } else {
        out.push_back(section);
      }
    }
    return out;
  }

private:
  // const 로 찾는다 (yaml-cpp 의 operator[] 는 const 가 아니면 없는 키를 만든다)
  static YAML::Node child(const YAML::Node & n, const std::string & key)
  {
    // 없는 키의 node 는 IsDefined() 말고는 아무것도 물을 수 없다 (예외). 그래서 없으면 Undefined node 로 바꿔 준다
    if (n.IsDefined() && n.IsMap()) {
      const YAML::Node & c = n;
      const YAML::Node value = c[key];
      if (value.IsDefined()) {
        return value;
      }
    }
    return YAML::Node(YAML::NodeType::Undefined);
  }

  YAML::Node find(const std::string & section, const std::string & key) const
  {
    return child(child(root_, section), key);
  }

  YAML::Node findStage(const std::string & stage, const std::string & key) const
  {
    const YAML::Node n = child(child(child(root_, "stages"), stage), key);
    return n.IsDefined() ? n : find("motion", key);
  }

  YAML::Node root_;
  std::string path_;
};

}  // namespace bt_mission

#endif  // BT_MISSION__DOCKING_CONFIG_HPP_
