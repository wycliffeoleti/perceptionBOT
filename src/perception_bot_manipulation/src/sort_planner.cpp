#include "perception_bot_manipulation/sort_planner.hpp"

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <utility>

namespace perception_bot {

namespace mtc = moveit::task_constructor;

std::string class_to_bin(const std::string &cls) {
  if (cls.find("red_") == 0) return "red";
  if (cls.find("blue_") == 0) return "blue";
  if (cls.find("green_") == 0) return "green";
  return "";
}

SortPlannerConfig default_config() {
  SortPlannerConfig cfg;

  auto make_pose = [](double x, double y, double z) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "world";
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);
    p.pose.orientation = tf2::toMsg(q);
    return p;
  };

  // 5cm above bin floor: bin floor is at z = 0.81 (table top), bin walls 8cm tall.
  cfg.place_poses["red"]   = make_pose(0.65, -0.25, 0.86);
  cfg.place_poses["blue"]  = make_pose(0.65,  0.00, 0.86);
  cfg.place_poses["green"] = make_pose(0.65,  0.25, 0.86);

  return cfg;
}

std::unique_ptr<mtc::Task> build_pick_place_task(
    const rclcpp::Node::SharedPtr &node,
    const SortPlannerConfig &cfg,
    const perception_bot_msgs::msg::DetectedObject &obj,
    const std::string &object_id) {
  std::string bin = class_to_bin(obj.class_name);
  if (bin.empty() || cfg.place_poses.find(bin) == cfg.place_poses.end()) {
    return nullptr;
  }
  const auto &place_pose = cfg.place_poses.at(bin);

  auto task = std::make_unique<mtc::Task>();
  task->stages()->setName("pick_place_" + object_id);
  task->loadRobotModel(node);

  task->setProperty("group", cfg.arm_group);
  task->setProperty("eef", cfg.eef_name);
  task->setProperty("hand", cfg.hand_group);
  task->setProperty("hand_grasping_frame", cfg.hand_frame);
  task->setProperty("ik_frame", cfg.hand_frame);

  // Solvers
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  sampling_planner->setPlannerId("ompl", "RRTConnect");
  sampling_planner->setProperty("max_velocity_scaling_factor", 0.5);
  sampling_planner->setProperty("max_acceleration_scaling_factor", 0.3);

  auto interp_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cart_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cart_planner->setMaxVelocityScalingFactor(0.5);
  cart_planner->setMaxAccelerationScalingFactor(0.3);
  cart_planner->setStepSize(cfg.cart_step);

  // ── 1. Current state ──
  auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
  mtc::Stage *current_state_ptr = current_state.get();
  task->add(std::move(current_state));

  // ── 2. Open hand ──
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interp_planner);
    stage->setGroup(cfg.hand_group);
    stage->setGoal(cfg.open_state);
    task->add(std::move(stage));
  }

  // ── 3. Connect to grasp pose ──
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to grasp",
        mtc::stages::Connect::GroupPlannerVector{{cfg.arm_group, sampling_planner}});
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task->add(std::move(stage));
  }

  // ── 4. Pick container (serial container) ──
  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick");
    task->properties().exposeTo(pick->properties(),
        {"eef", "hand", "group", "ik_frame"});
    pick->properties().configureInitFrom(mtc::Stage::PARENT,
        {"eef", "hand", "group", "ik_frame"});

    // 4a. Approach
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
          "approach object", cart_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", cfg.hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, cfg.approach_distance);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = cfg.world_frame;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    // 4b. Generate grasp pose
    mtc::Stage *grasp_pose_ptr = nullptr;
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(cfg.open_state);
      stage->setObject(object_id);
      stage->setAngleDelta(M_PI / 12);  // 15° steps
      stage->setMonitoredStage(current_state_ptr);

      // Top-down: rotate hand to point z-axis along world -Z.
      Eigen::Isometry3d grasp_offset = Eigen::Isometry3d::Identity();
      // Hand tcp z+ points away from wrist (along approach). For top-down we
      // want it along -world_Z, achieved when grasp_pose has roll=π.
      // GenerateGraspPose samples around the object's z-axis; by default the
      // sampled pose's z-axis matches the object's z (which is world Z+ for
      // top-resting objects). So we offset by π about X to flip.
      grasp_offset.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
                                  .toRotationMatrix();

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "grasp IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(0.5);
      wrapper->setIKFrame(grasp_offset, cfg.hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT,
          {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE,
          {"target_pose"});
      grasp_pose_ptr = wrapper.get();
      pick->insert(std::move(wrapper));
    }
    (void)grasp_pose_ptr;

    // 4c. Allow collision (object, hand)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (object,hand)");
      stage->allowCollisions(object_id,
          task->getRobotModel()->getJointModelGroup(cfg.hand_group)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      pick->insert(std::move(stage));
    }

    // 4d. Close hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
          "close hand", interp_planner);
      stage->setGroup(cfg.hand_group);
      stage->setGoal(cfg.close_state);
      pick->insert(std::move(stage));
    }

    // 4e. Attach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_id, cfg.hand_frame);
      pick->insert(std::move(stage));
    }

    // 4f. Lift
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
          "lift object", cart_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.03, cfg.lift_distance);
      stage->setIKFrame(cfg.hand_frame);
      stage->properties().set("marker_ns", "lift_object");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = cfg.world_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      pick->insert(std::move(stage));
    }

    task->add(std::move(pick));
  }

  // ── 5. Connect to place pose ──
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{cfg.arm_group, sampling_planner}});
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task->add(std::move(stage));
  }

  // ── 6. Place container ──
  {
    auto place = std::make_unique<mtc::SerialContainer>("place");
    task->properties().exposeTo(place->properties(),
        {"eef", "hand", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT,
        {"eef", "hand", "group", "ik_frame"});

    // 6a. Lower above bin
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
          "lower over bin", cart_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.02, 0.05);
      stage->setIKFrame(cfg.hand_frame);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = cfg.world_frame;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // 6b. Generate place pose
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT,
          {"ik_frame"});
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(object_id);

      geometry_msgs::msg::PoseStamped target = place_pose;
      stage->setPose(target);
      stage->setMonitoredStage(current_state_ptr);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
          "place IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setIKFrame(cfg.hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT,
          {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE,
          {"target_pose"});
      place->insert(std::move(wrapper));
    }

    // 6c. Open hand
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
          "open hand", interp_planner);
      stage->setGroup(cfg.hand_group);
      stage->setGoal(cfg.open_state);
      place->insert(std::move(stage));
    }

    // 6d. Disallow collision again
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "forbid collision (object,hand)");
      stage->allowCollisions(object_id,
          task->getRobotModel()->getJointModelGroup(cfg.hand_group)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    // 6e. Detach
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_id, cfg.hand_frame);
      place->insert(std::move(stage));
    }

    // 6f. Retreat
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
          "retreat", cart_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, cfg.retreat_distance);
      stage->setIKFrame(cfg.hand_frame);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = cfg.world_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task->add(std::move(place));
  }

  // ── 7. Return to home ──
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>(
        "return home", sampling_planner);
    stage->setGroup(cfg.arm_group);
    stage->setGoal(cfg.home_state);
    task->add(std::move(stage));
  }

  return task;
}

}  // namespace perception_bot
