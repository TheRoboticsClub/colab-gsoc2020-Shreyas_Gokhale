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
#include <map>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{

class Move : public BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit Move(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("goal")
    };
  }

private:
  int goal_reached_;
  std::map<std::string, geometry_msgs::msg::Pose2D> waypoints_;
};

} 
