// docking.yaml 읽기 (docking_config.hpp) 시험
#include <algorithm>
#include <string>

#include "gtest/gtest.h"
#include "bt_mission/docking_config.hpp"

using bt_mission::DockingConfig;

TEST(DockingConfig, ShippedFileHasNoUnknownKeysAndTheDockedDistance)
{
  std::string error;
  const auto c = DockingConfig::fromFile(DOCKING_YAML, &error);
  ASSERT_TRUE(c.loaded()) << error;
  EXPECT_TRUE(c.unknownKeys().empty()) << c.unknownKeys().front();
  const auto dock = c.dock("dock_a");
  ASSERT_TRUE(dock);
  EXPECT_EQ(dock->marker_id, 0);
  EXPECT_NEAR(dock->size, 0.06, 1e-12);
  // 붙었을 때: 로봇 앞면이 도킹 면 0.15 m 앞 = 로봇 중심은 0.15 + 0.16
  EXPECT_NEAR(c.dockedDistance("dock_a"), 0.31, 1e-12);
  EXPECT_FALSE(c.dock("no_such_dock"));
}

TEST(DockingConfig, StageOverridesMotionAndCodeDefaultsFillTheRest)
{
  const auto c = DockingConfig::fromString(
    "motion: {max_vx: 0.2, min_vy: 0.02, tol_yaw_deg: 0.5}\n"
    "stages:\n"
    "  insert: {max_vx: 0.05, hold_if_closer: true}\n");
  EXPECT_NEAR(c.mecanum("insert").max_vx, 0.05, 1e-12);        // 단계의 값
  EXPECT_NEAR(c.mecanum("approach").max_vx, 0.2, 1e-12);       // 단계에 없으면 motion
  EXPECT_NEAR(c.mecanum("insert").min_vy, 0.02, 1e-12);
  EXPECT_NEAR(c.mecanum("insert").tol_yaw, 0.5 * M_PI / 180.0, 1e-12);
  EXPECT_NEAR(c.mecanum("insert").k_lat, bt_mission::aruco_dock::MecanumParams().k_lat, 1e-12);   // 없으면 코드
  EXPECT_TRUE(c.stageFlag("insert", "hold_if_closer", false));
  EXPECT_FALSE(c.stageFlag("approach", "hold_if_closer", false));
  EXPECT_NEAR(c.stageNumber("insert", "standoff", 0.3), 0.3, 1e-12);
}

TEST(DockingConfig, MissingFileGivesDefaultsAndTyposAreReported)
{
  std::string error;
  const auto missing = DockingConfig::fromFile("/no/such/docking.yaml", &error);
  EXPECT_FALSE(missing.loaded());
  EXPECT_FALSE(error.empty());
  EXPECT_NEAR(missing.number("robot", "turn_radius", 0.22), 0.22, 1e-12);
  EXPECT_NEAR(missing.dockedDistance("dock_a"), 0.31, 1e-12);
  EXPECT_EQ(missing.text("ros", "scan_topic", "scan"), "scan");

  const auto c = DockingConfig::fromString(
    "motion: {max_vxx: 1.0}\nstages: {insert: {standof: 0.1}}\ndocks: {a: {sizee: 1}}\nros: {scan_topic: }\nextra: 1\n");
  const auto unknown = c.unknownKeys();
  for (const std::string key : {"motion.max_vxx", "stages.insert.standof", "docks.a.sizee", "extra"}) {
    EXPECT_NE(std::find(unknown.begin(), unknown.end(), key), unknown.end()) << key;
  }
  EXPECT_EQ(unknown.size(), 4u);
  EXPECT_EQ(c.text("ros", "scan_topic", "scan"), "");        // 일부러 비운 것은 빈 값 (레이저 없음)
  EXPECT_EQ(c.numbers("detect", "search_angles_deg", {30.0}).size(), 1u);
}

TEST(DockingConfig, ADockCanCarrySeveralMarkers)
{
  std::string error;
  const auto shipped = DockingConfig::fromFile(DOCKING_YAML, &error);
  ASSERT_TRUE(shipped.loaded()) << error;
  EXPECT_EQ(shipped.dock("dock_a")->marker_ids, std::vector<int>({0}));
  const auto two = shipped.dock("dock_b");
  ASSERT_TRUE(two);
  EXPECT_EQ(two->marker_ids, std::vector<int>({2, 3}));
  EXPECT_NEAR(shipped.dockedDistance("dock_b"), 0.31, 1e-12);
  // markers 안의 오타도 찾는다
  const auto c = DockingConfig::fromString(
    "docks:\n"
    "  d: {size: 0.06, height: 0.13, gap: 0.2, markers: [{id: 4, lateral: -0.08}, {id: 5, laterl: 0.08}]}\n");
  EXPECT_EQ(c.dock("d")->marker_ids, std::vector<int>({4, 5}));
  ASSERT_EQ(c.unknownKeys().size(), 1u);
  EXPECT_EQ(c.unknownKeys().front(), "docks.d.markers.laterl");
}
