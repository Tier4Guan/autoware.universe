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
#include <behavior_path_planner/include/behavior_path_planner/planner_manager.hpp>

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
{
  using std::placeholders::_1;

  // data_manager
  {
    planner_data_ = std::make_shared<PlannerData>();
    planner_data_->parameters = getCommonParam();
    planner_data_->drivable_area_expansion_parameters.init(*this);
  }

  // Subscriber
  route_sub_ = this->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "/planning/mission_planning/route", 1,
    std::bind(&BehaviorMsgConverterNode::onTrigger, this, _1), createSubscriptionOptions(this));
  trigger_sub_path_with_lane_id_ =
    this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>(
      "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", 1,
      std::bind(&BehaviorMsgConverterNode::onTrigger, this, _1), createSubscriptionOptions(this));

  // Publishers
  pathwithlaneid_pub_ = this->create_publisher<autoware_auto_planning_msgs::msg::PathWithLaneId>(
    "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", 1);
  path_pub_ = create_publisher<PathWithLaneId>("/planning/scenario_planning/lane_driving/behavior_planning/path", 1);
  
 // route_handler
  {
    const auto & p = planner_data_->parameters;
    planner_manager_ = std::make_shared<PlannerManager>(*this, p.verbose);
  }
}


// Callback

// Converting Route to PathWithLaneID 
void BehaviorMsgConverterNode::run(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr input_route_msg)
{
  // run behavior planner
  const auto output = planner_manager_->run(planner_data_);
  // path handling
  const auto path = getPath(output, planner_data_, planner_manager_);
  // update planner data
  planner_data_->prev_output_path = path;
}

PathWithLaneId::SharedPtr BehaviorMsgConverterNode::getPath(
  const BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & planner_data,
  const std::shared_ptr<PlannerManager> & planner_manager)
{
  auto path = output.path ? output.path : planner_data->prev_output_path;
  path->header = planner_data->route_handler->getRouteHeader();
  path->header.stamp = this->now();

  PathWithLaneId connected_path;
  const auto module_status_ptr_vec = planner_manager->getSceneModuleStatus();

  const auto resampled_path = utils::resamplePathWithSpline(
    *path, planner_data->parameters.output_path_interval, keepInputPoints(module_status_ptr_vec));
  return std::make_shared<PathWithLaneId>(resampled_path);
}



// Converting PathWithLaneID to Path
void BehaviorMsgConverterNode::onTrigger(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg)
{
  if (input_path_msg->points.empty()) {
    return;
  }

  const autoware_auto_planning_msgs::msg::Path output_path_msg = generatePath(input_path_msg);

  path_pub_->publish(output_path_msg);
}

autoware_auto_planning_msgs::msg::Path BehaviorMsgConverterNode::generatePath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg)
{
  autoware_auto_planning_msgs::msg::Path output_path_msg;

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
