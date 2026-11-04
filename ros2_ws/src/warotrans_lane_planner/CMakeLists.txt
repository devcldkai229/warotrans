cmake_minimum_required(VERSION 3.8)
project(warotrans_lane_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(warotrans_lane_planner SHARED
  src/lane_aware_navfn.cpp
)
target_include_directories(warotrans_lane_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(warotrans_lane_planner
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  pluginlib
  tf2
  tf2_ros
)
target_link_libraries(warotrans_lane_planner nlohmann_json::nlohmann_json)
target_compile_features(warotrans_lane_planner PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(nav2_core lane_aware_navfn_plugin.xml)

install(TARGETS warotrans_lane_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES lane_aware_navfn_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(warotrans_lane_planner)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util
  nav_msgs geometry_msgs pluginlib tf2 tf2_ros
)
ament_package()
