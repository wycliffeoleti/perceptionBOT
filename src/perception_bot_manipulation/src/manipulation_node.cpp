// manipulation_node — exposes the SortObjects action.
//
// On goal:
//   - Take the latest /perception/objects_world snapshot (subscriber).
//   - For each detection, add a CollisionObject to the planning scene
//     describing the object, then build & execute an MTC pick-place task,
//     finally remove the object.
//   - Stream feedback per object, return aggregated result.
#include "perception_bot_manipulation/sort_planner.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <perception_bot_msgs/action/sort_objects.hpp>
#include <perception_bot_msgs/msg/detected_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace perception_bot {

class ManipulationNode {
public:
  using SortObjects = perception_bot_msgs::action::SortObjects;
  using GoalHandleSort = rclcpp_action::ServerGoalHandle<SortObjects>;

  explicit ManipulationNode(rclcpp::Node::SharedPtr node)
      : node_(std::move(node)) {
    cfg_ = default_config();

    sub_ = node_->create_subscription<
        perception_bot_msgs::msg::DetectedObjectArray>(
        "/perception/objects_world", 10,
        [this](perception_bot_msgs::msg::DetectedObjectArray::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          last_ = std::move(msg);
        });

    server_ = rclcpp_action::create_server<SortObjects>(
        node_, "/sort_objects",
        std::bind(&ManipulationNode::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&ManipulationNode::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&ManipulationNode::handle_accept, this,
                  std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
        "manipulation_node ready — action /sort_objects");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const SortObjects::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleSort>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accept(const std::shared_ptr<GoalHandleSort> goal_handle) {
    std::thread([this, goal_handle] { execute(goal_handle); }).detach();
  }

  static moveit_msgs::msg::CollisionObject to_collision_object(
      const perception_bot_msgs::msg::DetectedObject &d,
      const std::string &id) {
    moveit_msgs::msg::CollisionObject co;
    co.header = d.pose.header;
    co.id = id;
    shape_msgs::msg::SolidPrimitive prim;
    if (d.class_name.find("cylinder") != std::string::npos) {
      prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      prim.dimensions.resize(2);
      // [height, radius]
      prim.dimensions[0] = std::max(0.04f, d.dimensions[2]);
      prim.dimensions[1] =
          std::max(0.015f, std::min(d.dimensions[0], d.dimensions[1]) * 0.5f);
    } else {
      prim.type = shape_msgs::msg::SolidPrimitive::BOX;
      prim.dimensions.resize(3);
      prim.dimensions[0] = std::max(0.03f, d.dimensions[0]);
      prim.dimensions[1] = std::max(0.03f, d.dimensions[1]);
      prim.dimensions[2] = std::max(0.03f, d.dimensions[2]);
    }
    co.primitives.push_back(prim);
    co.primitive_poses.push_back(d.pose.pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    return co;
  }

  void execute(const std::shared_ptr<GoalHandleSort> gh) {
    auto result = std::make_shared<SortObjects::Result>();
    auto feedback = std::make_shared<SortObjects::Feedback>();

    perception_bot_msgs::msg::DetectedObjectArray::ConstSharedPtr snap;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      snap = last_;
    }
    if (!snap || snap->objects.empty()) {
      result->objects_failed = 0;
      result->objects_sorted = 0;
      result->failure_reasons.push_back("no detections available");
      gh->abort(result);
      return;
    }

    auto goal = gh->get_goal();
    size_t n = snap->objects.size();
    if (goal->max_objects > 0 &&
        static_cast<size_t>(goal->max_objects) < n) {
      n = goal->max_objects;
    }
    feedback->total_objects = n;

    moveit::planning_interface::PlanningSceneInterface psi;

    auto t_start = node_->now();

    for (size_t i = 0; i < n; ++i) {
      const auto &obj = snap->objects[i];
      std::string id = "object_" + std::to_string(i);

      feedback->current_object_index = i;
      feedback->current_object_class = obj.class_name;
      feedback->current_action = "planning";
      gh->publish_feedback(feedback);

      // Add object to planning scene
      psi.applyCollisionObject(to_collision_object(obj, id));

      auto task = build_pick_place_task(node_, cfg_, obj, id);
      if (!task) {
        result->objects_failed++;
        result->failure_reasons.push_back(
            "no bin mapping for class " + obj.class_name);
        // Remove the placeholder object
        moveit_msgs::msg::CollisionObject rm;
        rm.id = id;
        rm.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        psi.applyCollisionObject(rm);
        continue;
      }

      try {
        task->init();
      } catch (const std::exception &e) {
        result->objects_failed++;
        result->failure_reasons.push_back(
            std::string("task init: ") + e.what());
        continue;
      }

      feedback->current_action = "picking";
      gh->publish_feedback(feedback);

      if (!task->plan(5)) {
        result->objects_failed++;
        result->failure_reasons.push_back("plan failed for " + obj.class_name);
        continue;
      }

      feedback->current_action = "executing";
      gh->publish_feedback(feedback);

      auto exec_result = task->execute(*task->solutions().front());
      if (exec_result.val !=
          moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        result->objects_failed++;
        result->failure_reasons.push_back(
            "execution failed for " + obj.class_name);
        continue;
      }

      result->objects_sorted++;
    }

    auto t_end = node_->now();
    result->total_time_seconds = (t_end - t_start).seconds();

    if (result->objects_failed == 0) {
      gh->succeed(result);
    } else {
      gh->abort(result);
    }
  }

  rclcpp::Node::SharedPtr node_;
  SortPlannerConfig cfg_;
  rclcpp_action::Server<SortObjects>::SharedPtr server_;
  rclcpp::Subscription<perception_bot_msgs::msg::DetectedObjectArray>::SharedPtr
      sub_;
  std::mutex mtx_;
  perception_bot_msgs::msg::DetectedObjectArray::ConstSharedPtr last_;
};

}  // namespace perception_bot

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // The node is created with use_intra_process_comms=false because MoveIt
  // services use a separate thread pool. Multi-threaded executor is needed
  // so action callbacks don't block the planning-scene callbacks.
  auto node = std::make_shared<rclcpp::Node>(
      "manipulation_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  perception_bot::ManipulationNode app(node);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
