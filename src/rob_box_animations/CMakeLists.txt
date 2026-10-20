cmake_minimum_required(VERSION 3.8)
project(rob_box_animations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# Python dependencies (sensor_msgs, std_msgs, rob_box_dialog_msgs, numpy) — package.xml

# Python package: procedural face (face/), director, face_node
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/face_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
