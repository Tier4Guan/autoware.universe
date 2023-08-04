// Copyright 2019 Autoware Foundation
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include <behavior_velocity_planner_common/planner_data.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_api_msgs/msg/crosswalk_status.hpp>
#include <tier4_api_msgs/msg/intersection_status.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>

namespace behavior_msg_converter
{
public:
  explicit BehaviorMsgConverterNode(const rclcpp::NodeOptions & node_options);

private:
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  // subscriber
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>::SharedPtr
    trigger_sub_path_with_lane_id_;
    
  void onTrigger(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg);
    
  // publisher
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Path>::SharedPtr path_pub_;
  
  // member
  PlannerData planner_data_;
  
  // function
  autoware_auto_planning_msgs::msg::Path generatePath(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg,
    const PlannerData & planner_data);

}  // namespace behavior_msg_converter

#endif  // NODE_HPP_
