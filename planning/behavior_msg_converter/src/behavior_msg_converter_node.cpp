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
#include <lanelet2_extension/utility/utilities.hpp>
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

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
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

namespace route_handler
{

RouteHandler::RouteHandler(const HADMapBin & map_msg)
{
  setMap(map_msg);
  route_ptr_ = nullptr;
}

void RouteHandler::setMap(const HADMapBin & map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  const lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  const lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  const lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);

  is_map_msg_ready_ = true;
  is_handler_ready_ = false;

  setLaneletsFromRouteMsg();
}

void RouteHandler::setRoute(const LaneletRoute & route_msg)
{
  if (!isRouteLooped(route_msg.segments)) {
    // if get not modified route but new route, reset original start pose
    if (!route_ptr_ || route_ptr_->uuid != route_msg.uuid) {
      original_start_pose_ = route_msg.start_pose;
      original_goal_pose_ = route_msg.goal_pose;
    }
    route_ptr_ = std::make_shared<LaneletRoute>(route_msg);
    is_handler_ready_ = false;
    setLaneletsFromRouteMsg();
  } else {
    RCLCPP_ERROR(
      logger_,
      "Loop detected within route! Currently, no loop is allowed for route! Using previous route");
  }
}

void RouteHandler::setLaneletsFromRouteMsg()
{
  if (!route_ptr_ || !is_map_msg_ready_) {
    return;
  }
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  const bool is_route_valid = lanelet::utils::route::isRouteValid(*route_ptr_, lanelet_map_ptr_);
  if (!is_route_valid) {
    return;
  }

  size_t primitive_size{0};
  for (const auto & route_section : route_ptr_->segments) {
    primitive_size += route_section.primitives.size();
  }
  route_lanelets_.reserve(primitive_size);

  for (const auto & route_section : route_ptr_->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      route_lanelets_.push_back(llt);
      if (id == route_section.preferred_primitive.id) {
        preferred_lanelets_.push_back(llt);
      }
    }
  }
  goal_lanelets_.clear();
  start_lanelets_.clear();
  if (!route_ptr_->segments.empty()) {
    goal_lanelets_.reserve(route_ptr_->segments.back().primitives.size());
    for (const auto & primitive : route_ptr_->segments.back().primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      goal_lanelets_.push_back(llt);
    }
    start_lanelets_.reserve(route_ptr_->segments.front().primitives.size());
    for (const auto & primitive : route_ptr_->segments.front().primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      start_lanelets_.push_back(llt);
    }
  }
  is_handler_ready_ = true;
}

bool RouteHandler::isRouteLooped(const RouteSections & route_sections)
{
  std::set<lanelet::Id> lane_primitives;
  for (const auto & route_section : route_sections) {
    for (const auto & primitive : route_section.primitives) {
      if (lane_primitives.find(primitive.id) != lane_primitives.end()) {
        return true;  // find duplicated id
      }
      lane_primitives.emplace(primitive.id);
    }
  }
  return false;
}

}  // namespace route_handler

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
  trigger_sub_path_with_lane_id_ =
    this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>(
      "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", 1,
      std::bind(&BehaviorMsgConverterNode::onTrigger, this, _1), createSubscriptionOptions(this));

  // Publishers
  pathwithlaneid_pub_ = this->create_publisher<autoware_auto_planning_msgs::msg::PathWithLaneId>(
    "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", 1);
  path_pub_ = create_publisher<PathWithLaneId>(
    "/planning/scenario_planning/lane_driving/behavior_planning/path", 1);

  // route_handler
  auto qos_transient_local = rclcpp::QoS{1}.transient_local();
  route_subscriber_ = create_subscription<LaneletRoute>(
    "~/input/route", qos_transient_local, std::bind(&BehaviorPathPlannerNode::onRoute, this, _1),
    createSubscriptionOptions(this));

  {
    const auto & p = planner_data_->parameters;
    planner_manager_ = std::make_shared<PlannerManager>(*this, p.verbose);
  }

  // Start timer
  {
    const auto planning_hz = declare_parameter<double>("planning_hz");
    const auto period_ns = rclcpp::Rate(planning_hz).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&BehaviorMsgConverterNode::run, this));
  }
}

PlannerParameters BehaviorMsgConverterNode::getCommonParam()
{
  PlannerParameters p{};

  p.verbose = declare_parameter<bool>("verbose");

  const auto get_scene_module_manager_param = [&](std::string && ns) {
    ModuleConfigParameters config;
    config.enable_module = declare_parameter<bool>(ns + "enable_module");
    config.enable_rtc = declare_parameter<bool>(ns + "enable_rtc");
    config.enable_simultaneous_execution_as_approved_module =
      declare_parameter<bool>(ns + "enable_simultaneous_execution_as_approved_module");
    config.enable_simultaneous_execution_as_candidate_module =
      declare_parameter<bool>(ns + "enable_simultaneous_execution_as_candidate_module");
    config.keep_last = declare_parameter<bool>(ns + "keep_last");
    config.priority = declare_parameter<int>(ns + "priority");
    config.max_module_size = declare_parameter<int>(ns + "max_module_size");
    return config;
  };

  p.config_start_planner = get_scene_module_manager_param("start_planner.");
  p.config_goal_planner = get_scene_module_manager_param("goal_planner.");
  p.config_side_shift = get_scene_module_manager_param("side_shift.");
  p.config_lane_change_left = get_scene_module_manager_param("lane_change_left.");
  p.config_lane_change_right = get_scene_module_manager_param("lane_change_right.");
  p.config_ext_request_lane_change_right =
    get_scene_module_manager_param("external_request_lane_change_right.");
  p.config_ext_request_lane_change_left =
    get_scene_module_manager_param("external_request_lane_change_left.");
  p.config_avoidance = get_scene_module_manager_param("avoidance.");
  p.config_avoidance_by_lc = get_scene_module_manager_param("avoidance_by_lc.");
  p.config_dynamic_avoidance = get_scene_module_manager_param("dynamic_avoidance.");

  // vehicle info
  const auto vehicle_info = VehicleInfoUtil(*this).getVehicleInfo();
  p.vehicle_info = vehicle_info;
  p.vehicle_width = vehicle_info.vehicle_width_m;
  p.vehicle_length = vehicle_info.vehicle_length_m;
  p.wheel_tread = vehicle_info.wheel_tread_m;
  p.wheel_base = vehicle_info.wheel_base_m;
  p.front_overhang = vehicle_info.front_overhang_m;
  p.rear_overhang = vehicle_info.rear_overhang_m;
  p.left_over_hang = vehicle_info.left_overhang_m;
  p.right_over_hang = vehicle_info.right_overhang_m;
  p.base_link2front = vehicle_info.max_longitudinal_offset_m;
  p.base_link2rear = p.rear_overhang;

  // NOTE: backward_path_length is used not only calculating path length but also calculating the
  // size of a drivable area.
  //       The drivable area has to cover not the base link but the vehicle itself. Therefore
  //       rear_overhang must be added to backward_path_length. In addition, because of the
  //       calculation of the drivable area in the obstacle_avoidance_planner package, the drivable
  //       area has to be a little longer than the backward_path_length parameter by adding
  //       min_backward_offset.
  constexpr double min_backward_offset = 1.0;
  const double backward_offset = vehicle_info.rear_overhang_m + min_backward_offset;

  // ROS parameters
  p.backward_path_length = declare_parameter<double>("backward_path_length") + backward_offset;
  p.forward_path_length = declare_parameter<double>("forward_path_length");

  // acceleration parameters
  p.min_acc = declare_parameter<double>("normal.min_acc");
  p.max_acc = declare_parameter<double>("normal.max_acc");

  // lane change parameters
  p.backward_length_buffer_for_end_of_lane =
    declare_parameter<double>("lane_change.backward_length_buffer_for_end_of_lane");
  p.lane_changing_lateral_jerk =
    declare_parameter<double>("lane_change.lane_changing_lateral_jerk");
  p.lane_change_prepare_duration = declare_parameter<double>("lane_change.prepare_duration");
  p.minimum_lane_changing_velocity =
    declare_parameter<double>("lane_change.minimum_lane_changing_velocity");
  p.minimum_lane_changing_velocity =
    std::min(p.minimum_lane_changing_velocity, p.max_acc * p.lane_change_prepare_duration);
  p.lane_change_finish_judge_buffer =
    declare_parameter<double>("lane_change.lane_change_finish_judge_buffer");

  // lateral acceleration map for lane change
  const auto lateral_acc_velocity =
    declare_parameter<std::vector<double>>("lane_change.lateral_acceleration.velocity");
  const auto min_lateral_acc =
    declare_parameter<std::vector<double>>("lane_change.lateral_acceleration.min_values");
  const auto max_lateral_acc =
    declare_parameter<std::vector<double>>("lane_change.lateral_acceleration.max_values");
  if (
    lateral_acc_velocity.size() != min_lateral_acc.size() ||
    lateral_acc_velocity.size() != max_lateral_acc.size()) {
    RCLCPP_ERROR(get_logger(), "Lane change lateral acceleration map has invalid size.");
    exit(EXIT_FAILURE);
  }
  for (size_t i = 0; i < lateral_acc_velocity.size(); ++i) {
    p.lane_change_lat_acc_map.add(
      lateral_acc_velocity.at(i), min_lateral_acc.at(i), max_lateral_acc.at(i));
  }

  p.backward_length_buffer_for_end_of_pull_over =
    declare_parameter<double>("backward_length_buffer_for_end_of_pull_over");
  p.backward_length_buffer_for_end_of_pull_out =
    declare_parameter<double>("backward_length_buffer_for_end_of_pull_out");

  p.minimum_pull_over_length = declare_parameter<double>("minimum_pull_over_length");
  p.refine_goal_search_radius_range = declare_parameter<double>("refine_goal_search_radius_range");
  p.turn_signal_intersection_search_distance =
    declare_parameter<double>("turn_signal_intersection_search_distance");
  p.turn_signal_intersection_angle_threshold_deg =
    declare_parameter<double>("turn_signal_intersection_angle_threshold_deg");
  p.turn_signal_minimum_search_distance =
    declare_parameter<double>("turn_signal_minimum_search_distance");
  p.turn_signal_search_time = declare_parameter<double>("turn_signal_search_time");
  p.turn_signal_shift_length_threshold =
    declare_parameter<double>("turn_signal_shift_length_threshold");
  p.turn_signal_on_swerving = declare_parameter<bool>("turn_signal_on_swerving");

  p.enable_akima_spline_first = declare_parameter<bool>("enable_akima_spline_first");
  p.enable_cog_on_centerline = declare_parameter<bool>("enable_cog_on_centerline");
  p.input_path_interval = declare_parameter<double>("input_path_interval");
  p.output_path_interval = declare_parameter<double>("output_path_interval");
  p.visualize_maximum_drivable_area = declare_parameter<bool>("visualize_maximum_drivable_area");
  p.ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
  p.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");

  if (p.backward_length_buffer_for_end_of_lane < 1.0) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Lane change buffer must be more than 1 meter. Modifying the buffer.");
  }
  return p;
}

// Callback

// Converting Route to PathWithLaneID
// get output as Reference Path
lanelet::ConstLanelet closest_lane
{
}
const auto output = getReferencePath(closest_lane, planner_data_);
// path handling
const auto path = getPath(output, planner_data_, planner_manager_);
// update planner data
planner_data_->prev_output_path = path;

BehaviorModuleOutput getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  PathWithLaneId reference_path{};

  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_odometry->pose.pose;
  const auto p = planner_data->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  // calculate path with backward margin to avoid end points' instability by spline interpolation
  constexpr double extra_margin = 10.0;
  const double backward_length = p.backward_path_length + extra_margin;
  const auto current_lanes_with_backward_margin =
    route_handler->getLaneletSequence(current_lane, backward_length, p.forward_path_length);
  const auto no_shift_pose =
    lanelet::utils::getClosestCenterPose(current_lane, current_pose.position);
  reference_path = getCenterLinePath(
    *route_handler, current_lanes_with_backward_margin, no_shift_pose, backward_length,
    p.forward_path_length, p);

  // clip backward length
  // NOTE: In order to keep backward_path_length at least, resampling interval is added to the
  // backward.
  const size_t current_seg_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    reference_path.points, no_shift_pose, p.ego_nearest_dist_threshold,
    p.ego_nearest_yaw_threshold);
  reference_path.points = motion_utils::cropPoints(
    reference_path.points, no_shift_pose.position, current_seg_idx, p.forward_path_length,
    p.backward_path_length + p.input_path_interval);

  const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);
  const auto drivable_lanes = generateDrivableLanes(drivable_lanelets);

  const auto & dp = planner_data->drivable_area_expansion_parameters;

  const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(reference_path);
  output.reference_path = std::make_shared<PathWithLaneId>(reference_path);
  output.drivable_area_info.drivable_lanes = drivable_lanes;

  return output;
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

void BehaviorMsgConverterNode::run()
{
  if (!isDataReady()) {
    return;
  }

  // RCLCPP_DEBUG(get_logger(), "----- BehaviorMsgConverterNode start -----");

  // check for map update
  HADMapBin::ConstSharedPtr map_ptr{nullptr};
  {
    std::lock_guard<std::mutex> lk_map(mutex_map_);  // for has_received_map_ and map_ptr_
    if (has_received_map_) {
      // Note: duplicating the shared_ptr prevents the data from being deleted by another thread!
      map_ptr = map_ptr_;
      has_received_map_ = false;
    }
  }

  // check for route update
  LaneletRoute::ConstSharedPtr route_ptr{nullptr};
  {
    std::lock_guard<std::mutex> lk_route(mutex_route_);  // for has_received_route_ and route_ptr_
    if (has_received_route_) {
      // Note: duplicating the shared_ptr prevents the data from being deleted by another thread!
      route_ptr = route_ptr_;
      has_received_route_ = false;
    }
  }

  std::unique_lock<std::mutex> lk_pd(mutex_pd_);  // for planner_data_

  // update map
  if (map_ptr) {
    planner_data_->route_handler->setMap(*map_ptr);
  }

  std::unique_lock<std::mutex> lk_manager(mutex_manager_);  // for planner_manager_

  // update route
  const bool is_first_time = !(planner_data_->route_handler->isHandlerReady());
  if (route_ptr) {
    planner_data_->route_handler->setRoute(*route_ptr);
    planner_manager_->resetRootLanelet(planner_data_);

    // uuid is not changed when rerouting with modified goal,
    // in this case do not need to rest modules.
    const bool has_same_route_id =
      planner_data_->prev_route_id && route_ptr->uuid == planner_data_->prev_route_id;
    // Reset behavior tree when new route is received,
    // so that the each modules do not have to care about the "route jump".
    if (!is_first_time && !has_same_route_id) {
      planner_manager_->reset();
    }
  }

  const auto controlled_by_autoware_autonomously =
    planner_data_->operation_mode->mode == OperationModeState::AUTONOMOUS &&
    planner_data_->operation_mode->is_autoware_control_enabled;
  if (!controlled_by_autoware_autonomously) {
    planner_manager_->resetRootLanelet(planner_data_);
  }

  // run behavior planner
  const auto output = planner_manager_->run(planner_data_);

  // path handling
  const auto path = getPath(output, planner_data_, planner_manager_);
  // update planner data
  planner_data_->prev_output_path = path;

  // compute turn signal
  computeTurnSignal(planner_data_, *path, output);

  // publish reroute availability
  publish_reroute_availability();

  // publish drivable bounds
  publish_bounds(*path);

  // NOTE: In order to keep backward_path_length at least, resampling interval is added to the
  // backward.
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  if (!path->points.empty()) {
    const size_t current_seg_idx = planner_data_->findEgoSegmentIndex(path->points);
    path->points = motion_utils::cropPoints(
      path->points, current_pose.position, current_seg_idx,
      planner_data_->parameters.forward_path_length,
      planner_data_->parameters.backward_path_length +
        planner_data_->parameters.input_path_interval);

    if (!path->points.empty()) {
      path_publisher_->publish(*path);
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "behavior path output is empty! Stop publish.");
    }
  } else {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "behavior path output is empty! Stop publish.");
  }

  publishSceneModuleDebugMsg(planner_manager_->getDebugMsg());
  publishPathCandidate(planner_manager_->getSceneModuleManagers(), planner_data_);
  publishPathReference(planner_manager_->getSceneModuleManagers(), planner_data_);
  stop_reason_publisher_->publish(planner_manager_->getStopReasons());

  // publish modified goal only when it is updated
  if (
    output.modified_goal &&
    /* has changed modified goal */ (
      !planner_data_->prev_modified_goal || tier4_autoware_utils::calcDistance2d(
                                              planner_data_->prev_modified_goal->pose.position,
                                              output.modified_goal->pose.position) > 0.01)) {
    PoseWithUuidStamped modified_goal = *(output.modified_goal);
    modified_goal.header.stamp = path->header.stamp;
    planner_data_->prev_modified_goal = modified_goal;
    modified_goal_publisher_->publish(modified_goal);
  }

  planner_data_->prev_route_id = planner_data_->route_handler->getRouteUuid();

  if (planner_data_->parameters.visualize_maximum_drivable_area) {
    const auto maximum_drivable_area = marker_utils::createFurthestLineStringMarkerArray(
      utils::getMaximumDrivableArea(planner_data_));
    debug_maximum_drivable_area_publisher_->publish(maximum_drivable_area);
  }

  lk_pd.unlock();  // release planner_data_

  planner_manager_->print();
  planner_manager_->publishMarker();
  planner_manager_->publishVirtualWall();
  lk_manager.unlock();  // release planner_manager_

  RCLCPP_DEBUG(get_logger(), "----- behavior path planner end -----\n\n");
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
