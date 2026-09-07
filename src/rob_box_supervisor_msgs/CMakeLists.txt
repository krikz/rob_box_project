cmake_minimum_required(VERSION 3.8)
project(rob_box_supervisor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages and service (ADR-0051 §2.1)
# - msg/Command.msg, msg/Response.msg
# - srv/ExecuteCommand.srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Command.msg"
  "msg/Response.msg"
  "srv/ExecuteCommand.srv"
  DEPENDENCIES
    std_msgs
)

ament_package()