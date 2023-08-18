// Copyright 2023 Tier IV, Inc.
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

// copied from behavior_velocity_planner
// Check whether a header is important, by deleting it and see if error.
#include "behavior_msg_converter_node.hpp"

#include <behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <tier4_autoware_utils/ros/wait_for_param.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_routing/Route.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace
{
rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node * node_ptr)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}
}  // namespace

// Converting PathWitLaneID to Path
namespace behavior_msg_converter
{
namespace
{

autoware_auto_planning_msgs::msg::Path to_path(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path_with_id)
{
  autoware_auto_planning_msgs::msg::Path path;
  for (const auto & path_point : path_with_id.points) {
    path.points.push_back(path_point.point);
  }
  return path;
}
}  // namespace

// Subscriber and publisher
BehaviorMsgConverterNode::BehaviorMsgConverterNode(const rclcpp::NodeOptions & node_options)
: Node("behavior_msg_converter_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
// planner_data_(*this)
{
  using std::placeholders::_1;
  // Trigger Subscriber
  trigger_sub_path_with_lane_id_ =
    this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>(
      // "~/input/path_with_lane_id", 1, std::bind(&BehaviorMsgConverterNode::onTrigger, this, _1),
      "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", 1,
      std::bind(&BehaviorMsgConverterNode::onTrigger, this, _1), createSubscriptionOptions(this));

  // Publishers
  // path_pub_ = this->create_publisher<autoware_auto_planning_msgs::msg::Path>("~/output/path", 1);
  path_pub_ = this->create_publisher<autoware_auto_planning_msgs::msg::Path>(
    "/planning/scenario_planning/lane_driving/behavior_planning/path", 1);
}

// Callback
void BehaviorMsgConverterNode::onTrigger(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg)
{
  if (input_path_msg->points.empty()) {
    return;
  }

  const autoware_auto_planning_msgs::msg::Path output_path_msg = generatePath(input_path_msg);
  // generatePath(input_path_msg, planner_data_);

  path_pub_->publish(output_path_msg);
}

autoware_auto_planning_msgs::msg::Path BehaviorMsgConverterNode::generatePath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg)
// const PlannerData & planner_data
{
  autoware_auto_planning_msgs::msg::Path output_path_msg;

  // Copied from behavior velocity planner.  Remember to fix this along with behavior velocity
  // planner.
  output_path_msg = to_path(*input_path_msg);
  output_path_msg.header.frame_id = "map";
  output_path_msg.header.stamp = this->now();
  output_path_msg.left_bound = input_path_msg->left_bound;
  output_path_msg.right_bound = input_path_msg->right_bound;
  RCLCPP_INFO(this->get_logger(), "Publishing");
  return output_path_msg;
}
}  // namespace behavior_msg_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_msg_converter::BehaviorMsgConverterNode)
