// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "amazon_robot_controller/robot_controller.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>
#include <set>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "amazon_robot_controller/ros_topic_logger.hpp"

namespace amazon_robot_controller
{

RobotController::RobotController()
: nav2_util::LifecycleNode("robot_controller", "", false),
  start_time_(0)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_change_goal_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_distance_traveled_condition_bt_node"
  };

  // Declare this node's parameters
  declare_parameter("default_bt_xml_filename");
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("robot_base_frame", std::string("base_link"));
  declare_parameter("odom_topic", std::string("odom"));
}

RobotController::~RobotController()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
RobotController::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // use suffix '_rclcpp_node' to keep parameter file consistency #1773
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r", std::string("__node:=") + get_name() + "_rclcpp_node",
      "--"});
  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "follow_targets", std::bind(&RobotController::followTargets, this));

  // Get the libraries to pull plugins from
  plugin_lib_names_ = get_parameter("plugin_lib_names").as_string_array();

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));  // NOLINT
  blackboard_->set<bool>("path_updated", false);  // NOLINT
  blackboard_->set<bool>("initial_pose_received", false);  // NOLINT
  blackboard_->set<int>("number_recoveries", 0);  // NOLINT
  blackboard_->set<int>("joint_position", 0);  // NOLINT

  // Get the BT filename to use from the node parameter
  std::string bt_xml_filename;
  get_parameter("bt_xml_filename", bt_xml_filename);

  if (!loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(get_logger(), "Error loading XML file: %s", bt_xml_filename.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

bool
RobotController::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return false;
  }

  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  RCLCPP_DEBUG(get_logger(), "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_DEBUG(get_logger(), "Behavior Tree XML: %s", xml_string.c_str());

  // Create the Behavior Tree from the XML input
  tree_ = bt_->buildTreeFromText(xml_string, blackboard_);

  return true;
}

nav2_util::CallbackReturn
RobotController::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  if (use_bond_) {
    // create bond connection
    createBond();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RobotController::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  if (use_bond_) {
    // destroy bond connection
    destroyBond();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RobotController::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // TODO(orduno) Fix the race condition between the worker thread ticking the tree
  //              and the main thread resetting the resources, see #1344
  client_node_.reset();

  action_server_.reset();
  plugin_lib_names_.clear();
  xml_string_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_.rootNode());
  bt_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
RobotController::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}



// ****************************************
// Code to edit






void
RobotController::followTargets()
{
  auto goal = action_server_->get_current_goal();
  if (goal->loadposepair.size() == 0) {
    RCLCPP_ERROR(get_logger(), "Goal has no pose. Terminating current goal.");
    action_server_->terminate_current();
    return;
  }

  // if (goal->load.size() == 0) {
  //   RCLCPP_ERROR(get_logger(), "Goal has no load. Terminating current goal.");
  //   action_server_->terminate_current();
  //   return;
  // }


  initializeBlackboard(goal);

  auto is_canceling = [this]() {
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable. Canceling.");
        return true;
      }

      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server is inactive. Canceling.");
        return true;
      }

      return action_server_->is_cancel_requested();
    };


  std::shared_ptr<Action::Feedback> feedback_msg = std::make_shared<Action::Feedback>();

  auto on_loop = [&]() {
      if (action_server_->is_preempt_requested()) {
        RCLCPP_INFO(get_logger(), "Received goal preemption request");
        action_server_->accept_pending_goal();
        auto goal = action_server_->get_current_goal();
        if (goal->loadposepair.size() == 0) {
          RCLCPP_ERROR(get_logger(), "Goal has no pose. Terminating current goal.");
          action_server_->terminate_current();
        }
        // else if (goal->load.size() == 0) {
        //   RCLCPP_ERROR(get_logger(), "Goal has no load. Terminating current goal.");
        //   action_server_->terminate_current();
        // }
        else {
          initializeBlackboard(goal);
        }
      }

      int current_waypoint_idx = 0;
      blackboard_->get<int>("current_waypoint_idx", current_waypoint_idx);
      feedback_msg->current_waypoint = current_waypoint_idx;
      action_server_->publish_feedback(feedback_msg);
    };


  // Execute the BT that was previously created in the configure step
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling);
  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not needed.
  bt_->haltAllActions(tree_.rootNode());

  switch (rc) {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      action_server_->succeeded_current();
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Navigation failed");
      action_server_->terminate_current();
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(get_logger(), "Navigation canceled");
      action_server_->terminate_all();
      break;

    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

void
RobotController::initializeBlackboard(std::shared_ptr<const Action::Goal> goal)
{
  // Update the goals on the blackboard
  blackboard_->set("goals", goal->loadposepair);
  blackboard_->set("current_waypoint_idx", 0);
  blackboard_->set("num_waypoints", goal->loadposepair.size());
  blackboard_->set("goal", goal->loadposepair[1].pose);
  blackboard_->set("goal_achieved", false);
}

}  // namespace amazon_robot_controller
