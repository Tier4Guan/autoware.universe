cmake_minimum_required(VERSION 3.14)
project(behavior_msg_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(behavior_msg_converter_node SHARED
  src/behavior_msg_converter_node.cpp
)

rclcpp_components_register_node(behavior_msg_converter_node
  PLUGIN "behavior_msg_converter::BehaviorMsgConverterNode"
  EXECUTABLE behavior_msg_converter
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
