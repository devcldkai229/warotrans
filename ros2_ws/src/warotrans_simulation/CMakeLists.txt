cmake_minimum_required(VERSION 3.8)
project(warotrans_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds maps urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/generate_warehouse.py scripts/motor_deadband_emulator.py scripts/nav_regression.py scripts/fetch_route_graph.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
