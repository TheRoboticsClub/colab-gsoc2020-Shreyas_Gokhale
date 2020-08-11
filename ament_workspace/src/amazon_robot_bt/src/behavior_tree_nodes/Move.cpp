// Copyright 2019 Intelligent Robotics Lab
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
// Modified By: Shreyas Gokhale <shreyas6gokhale@gmail.com>

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "amazon_robot_bt/Move.hpp"


namespace nav2_behavior_tree
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
  // rclcpp::Node::SharedPtr node;
  // config().blackboard->get("node", node);

  // node->declare_parameter("waypoints");
  // node->declare_parameter("waypoint_coords");

  // if (node->has_parameter("waypoints")) {
  //   std::vector<std::string> wp_names;

  //   node->get_parameter_or("waypoints", wp_names, {});

  //   for (auto & wp : wp_names) {
  //     node->declare_parameter("waypoint_coords." + wp);

  //     std::vector<double> coords;
  //     if (node->get_parameter_or("waypoint_coords." + wp, coords, {})) {
  //       geometry_msgs::msg::Pose2D pose;
  //       pose.x = coords[0];
  //       pose.y = coords[1];
  //       pose.theta = coords[2];

  //       waypoints_[wp] = pose;
  //     } else {
  //       std::cerr << "No coordinate configured for waypoint [" << wp << "]" << std::endl;
  //     }
  //   }
  // }

  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  node->declare_parameter("waypoints");
  node->declare_parameter("waypoint_coords");

  if (node->has_parameter("waypoints")) {
    std::vector<std::string> wp_names;

    node->get_parameter_or("waypoints", wp_names, {});

    for (auto & wp : wp_names) {
      node->declare_parameter("waypoint_coords." + wp);

      std::vector<double> coords;
      if (node->get_parameter_or("waypoint_coords." + wp, coords, {})) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = coords[0];
        pose.position.y = coords[1];
        // pose.point.theta = coords[2];

        waypoints_[wp] = pose;
      } else {
        std::cerr << "No coordinate configured for waypoint [" << wp << "]" << std::endl;
      }
    }
  }
}

void
Move::on_tick()
{
  if (!getInput("goal", goal_.pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateToPoseAction: goal not provided");
    return;
  }


  // std::string goal;
  // getInput<std::string>("goal", goal);

  // geometry_msgs::msg::Pose2D pose2nav;
  // if (waypoints_.find(goal) != waypoints_.end()) {
  //   pose2nav = waypoints_[goal];
  // } else {
  //   std::cerr << "No coordinate for waypoint [" << goal << "]" << std::endl;
  // }

  // geometry_msgs::msg::PoseStamped goal_pos;

  // goal_pos.header.frame_id = "map";
  // goal_pos.pose.position.x = pose2nav.x;
  // goal_pos.pose.position.y = pose2nav.y;
  // goal_pos.pose.position.z = 0;
  // goal_pos.pose.orientation = tf2::toMsg(tf2::Quaternion({0.0, 0.0, 1.0}, pose2nav.theta));

  // goal_.pose = goal_pos;
}

// BT::NodeStatus
// Move::on_success()
// {
//   return BT::NodeStatus::SUCCESS;
// }


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::Move>(
        name, "move_to_pallet", config);
    };

  factory.registerBuilder<nav2_behavior_tree::Move>(
    "Move", builder);
}
