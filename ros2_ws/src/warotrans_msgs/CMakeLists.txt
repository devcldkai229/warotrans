cmake_minimum_required(VERSION 3.8)
project(warotrans_msgs)

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Interface generation is intentionally NOT enabled yet: msg/, srv/ and action/
# are still empty, and rosidl_generate_interfaces() fails when given no files.
#
# The day the first interface file is added, uncomment the block below AND add
# the matching entries to package.xml (they are listed there, commented, in the
# same order). Doing only one of the two produces a build error whose message
# does not point at the real cause.
# ---------------------------------------------------------------------------
# find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/YourMessage.msg"
#   DEPENDENCIES std_msgs geometry_msgs
# )

ament_package()
