// Copyright 2023 Autoware Foundation
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

#ifndef BEHAVIOR_MSG_CONVERTER_NODE_HPP_
#define BEHAVIOR_MSG_CONVERTER_NODE_HPP_

// #include <behavior_velocity_planner_common/planner_data.hpp>
#include <behavior_velocity_planner_common/plugin_interface.hpp>
#include <behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

// #include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
// #include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_api_msgs/msg/crosswalk_status.hpp>
#include <tier4_api_msgs/msg/intersection_status.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_msg_converter
{

class BehaviorMsgConverterNode : public rclcpp::Node
{
public:
  explicit BehaviorMsgConverterNode(const rclcpp::NodeOptions & node_options);

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<PlannerData> planner_data_;
  std::shared_ptr<PlannerManager> planner_manager_;

  // subscriber
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>::SharedPtr
    trigger_sub_path_with_lane_id_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr route_sub_;

  void onTrigger(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg);

  // publisher
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<PathWithLaneId>::SharedPtr pathwithlaneid_pub_;

  // member
  // PlannerData planner_data_;

  // function
  autoware_auto_planning_msgs::msg::Path generatePath(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg);

  BehaviorModuleOutput getReferencePath(
    const lanelet::ConstLanelet & current_lane,
    const std::shared_ptr<const PlannerData> & planner_data);

  PathWithLaneId::SharedPtr getPath(
    const BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & planner_data,
    const std::shared_ptr<PlannerManager> & planner_manager);
};

struct PlannerData
{
  Odometry::ConstSharedPtr self_odometry{};
  AccelWithCovarianceStamped::ConstSharedPtr self_acceleration{};
  PredictedObjects::ConstSharedPtr dynamic_object{};
  OccupancyGrid::ConstSharedPtr occupancy_grid{};
  OccupancyGrid::ConstSharedPtr costmap{};
  LateralOffset::ConstSharedPtr lateral_offset{};
  OperationModeState::ConstSharedPtr operation_mode{};
  PathWithLaneId::SharedPtr reference_path{std::make_shared<PathWithLaneId>()};
  PathWithLaneId::SharedPtr prev_output_path{std::make_shared<PathWithLaneId>()};
  std::optional<PoseWithUuidStamped> prev_modified_goal{};
  std::optional<UUID> prev_route_id{};
  lanelet::ConstLanelets current_lanes{};
  std::shared_ptr<RouteHandler> route_handler{std::make_shared<RouteHandler>()};
  PlannerParameters parameters{};
  drivable_area_expansion::DrivableAreaExpansionParameters drivable_area_expansion_parameters{};

  mutable std::optional<geometry_msgs::msg::Pose> drivable_area_expansion_prev_crop_pose;
  mutable TurnSignalDecider turn_signal_decider;

  TurnIndicatorsCommand getTurnSignal(
    const PathWithLaneId & path, const TurnSignalInfo & turn_signal_info,
    TurnSignalDebugData & debug_data)
  {
    const auto & current_pose = self_odometry->pose.pose;
    const auto & current_vel = self_odometry->twist.twist.linear.x;
    return turn_signal_decider.getTurnSignal(
      route_handler, path, turn_signal_info, current_pose, current_vel, parameters, debug_data);
  }

  template <class T>
  size_t findEgoIndex(const std::vector<T> & points) const
  {
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      points, self_odometry->pose.pose, parameters.ego_nearest_dist_threshold,
      parameters.ego_nearest_yaw_threshold);
  }

  template <class T>
  size_t findEgoSegmentIndex(const std::vector<T> & points) const
  {
    return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      points, self_odometry->pose.pose, parameters.ego_nearest_dist_threshold,
      parameters.ego_nearest_yaw_threshold);
  }
};

}  // namespace behavior_msg_converter

namespace route_handler
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Header;
using unique_identifier_msgs::msg::UUID;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;

enum class Direction { NONE, LEFT, RIGHT };
enum class PullOverDirection { NONE, LEFT, RIGHT };
enum class PullOutDirection { NONE, LEFT, RIGHT };

class RouteHandler
{
public:
  RouteHandler() = default;
  explicit RouteHandler(const HADMapBin & map_msg);

  // non-const methods
  void setMap(const HADMapBin & map_msg);
  void setRoute(const LaneletRoute & route_msg);
  
  // for routing
  static bool isRouteLooped(const RouteSections & route_sections);
  
  private:
  // MUST
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  lanelet::ConstLanelets route_lanelets_;
  lanelet::ConstLanelets preferred_lanelets_;
  lanelet::ConstLanelets start_lanelets_;
  lanelet::ConstLanelets goal_lanelets_;
  lanelet::ConstLanelets shoulder_lanelets_;
  std::shared_ptr<LaneletRoute> route_ptr_{nullptr};

  rclcpp::Logger logger_{rclcpp::get_logger("route_handler")};

  bool is_map_msg_ready_{false};
  bool is_handler_ready_{false};
  
  // non-const methods
  void setLaneletsFromRouteMsg();
}  
  
}

#endif  // BEHAVIOR_MSG_CONVERTER_NODE_HPP_
