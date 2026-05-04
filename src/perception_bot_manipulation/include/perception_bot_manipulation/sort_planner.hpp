// Builds a MoveIt Task Constructor pick-and-place Task that picks the given
// object from the planning scene and places it into a color-matched bin.
#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/task.h>
#include <perception_bot_msgs/msg/detected_object.hpp>
#include <rclcpp/node.hpp>

#include <map>
#include <memory>
#include <string>

namespace perception_bot {

struct SortPlannerConfig {
  std::string arm_group = "fr3_arm";
  std::string hand_group = "fr3_hand";
  std::string eef_name = "fr3_hand";
  std::string hand_frame = "fr3_hand_tcp";
  std::string world_frame = "world";

  // Named SRDF states.
  std::string home_state = "ready";
  std::string open_state = "open";
  std::string close_state = "close";

  // Approach / lift / retreat distances along world Z.
  double approach_distance = 0.10;
  double lift_distance = 0.10;
  double retreat_distance = 0.10;

  // Place pose for each color/class — center of each bin in world frame.
  // Bins from sorting_world.sdf:
  //   bin_red   pose 0.65 -0.25 0.81
  //   bin_blue  pose 0.65  0.00 0.81
  //   bin_green pose 0.65  0.25 0.81
  // Place 5cm above the table surface inside the bin.
  std::map<std::string, geometry_msgs::msg::PoseStamped> place_poses;

  // Cartesian planner step / jump_threshold for approach/retreat.
  double cart_step = 0.005;
  double cart_jump_threshold = 0.0;

  // Pre-grasp Z offset above object center (m) when generating grasp pose.
  double grasp_offset_z = 0.0;
};

// Returns nullptr if a Task could not be constructed (e.g. unknown class).
std::unique_ptr<moveit::task_constructor::Task>
build_pick_place_task(
    const rclcpp::Node::SharedPtr &node,
    const SortPlannerConfig &cfg,
    const perception_bot_msgs::msg::DetectedObject &obj,
    const std::string &object_id);

// Build the default config with bin centers populated.
SortPlannerConfig default_config();

// Map detector class_name → place pose key. Returns "" if unknown.
std::string class_to_bin(const std::string &class_name);

}  // namespace perception_bot
