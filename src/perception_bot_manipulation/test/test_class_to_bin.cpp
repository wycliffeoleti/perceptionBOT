#include "perception_bot_manipulation/sort_planner.hpp"

#include <gtest/gtest.h>

TEST(ClassToBin, RoutesByPrefix) {
  EXPECT_EQ(perception_bot::class_to_bin("red_cube"), "red");
  EXPECT_EQ(perception_bot::class_to_bin("blue_cube"), "blue");
  EXPECT_EQ(perception_bot::class_to_bin("green_cylinder"), "green");
  EXPECT_EQ(perception_bot::class_to_bin("yellow_cube"), "");
  EXPECT_EQ(perception_bot::class_to_bin(""), "");
}

TEST(DefaultConfig, HasThreeBinPoses) {
  auto cfg = perception_bot::default_config();
  EXPECT_EQ(cfg.place_poses.size(), 3u);
  EXPECT_NE(cfg.place_poses.find("red"), cfg.place_poses.end());
  EXPECT_NE(cfg.place_poses.find("blue"), cfg.place_poses.end());
  EXPECT_NE(cfg.place_poses.find("green"), cfg.place_poses.end());
  for (const auto &kv : cfg.place_poses) {
    EXPECT_EQ(kv.second.header.frame_id, "world");
    EXPECT_NEAR(kv.second.pose.position.x, 0.65, 1e-6);
    EXPECT_NEAR(kv.second.pose.position.z, 0.86, 1e-6);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
