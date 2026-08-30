cmake_minimum_required(VERSION 3.8)
project(warotrans_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
