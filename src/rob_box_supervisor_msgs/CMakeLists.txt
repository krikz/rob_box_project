cmake_minimum_required(VERSION 3.8)
project(rob_box_supervisor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate messages and services
# Field-level rationale lives next to each .srv / .msg file.
# Field names mirror meta-quest-api.md §3 (frame types 0x31–0x32) and
# ADR-0028 §4.2 (LockManager), §4.3 (ROS 2 API), §4.4 (client API).
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AcquireFloor.srv"
  "srv/ReleaseFloor.srv"
  "srv/SetAvatarMode.srv"
  "msg/TeleopHeartbeat.msg"
  "msg/FloorState.msg"
  "msg/AvatarStateMsg.msg"
  DEPENDENCIES
    std_msgs
    builtin_interfaces
)

ament_package()
