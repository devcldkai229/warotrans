cmake_minimum_required(VERSION 3.8)
project(warotrans_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds maps urdf behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/generate_warehouse.py scripts/motor_deadband_emulator.py scripts/nav_regression.py scripts/fetch_route_graph.sh
          scripts/sync_web_map.py scripts/spawn_pose.py scripts/measure_landmarks.py scripts/sim_init_gate.py
          scripts/build_map_one_sim_bundle.py scripts/stage1_open_grant_node.py
          scripts/semantic_map_adapter.py scripts/write_robot_bridge_yaml.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.py"
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_semantic_map_adapter test/test_semantic_map_adapter.py)
  ament_add_pytest_test(test_lane_aware_plugin_id test/test_lane_aware_plugin_id.py)
endif()

ament_package()
