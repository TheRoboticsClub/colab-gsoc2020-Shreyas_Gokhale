// Copyright (c) 2020 ymd-stella
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

#include <string>
#include <vector>

#include "amazon_robot_controller/plugins/action/get_next_goal_action.hpp"

namespace amazon_robot_controller
{

GetNextGoalAction::GetNextGoalAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(action_name, conf)
{
}

BT::NodeStatus GetNextGoalAction::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (!getInput("goals", goals)) {
    return BT::NodeStatus::FAILURE;
  }
  int64_t current_waypoint_idx = config().blackboard->get<int64_t>("current_waypoint_idx");
  int64_t num_waypoints = config().blackboard->get<int64_t>("num_waypoints");

  bool goal_achieved;
  if (!getInput("goal_achieved", goal_achieved)) {
    goal_achieved = false;
  }

  if (!goal_achieved) {
    return BT::NodeStatus::SUCCESS;
  }

  if (current_waypoint_idx >= num_waypoints - 1) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("goal_achieved", false);
  // setOutput("goal", goals.second[current_waypoint_idx + 1]);
  setOutput("goal", goals[current_waypoint_idx + 1]);
  config().blackboard->set("current_waypoint_idx", current_waypoint_idx + 1);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace amazon_robot_controller

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<amazon_robot_controller::GetNextGoalAction>("GetNextGoal");
}
