cmake_minimum_required(VERSION 3.8)
project(rob_box_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Модули без ROS (миры, геометрия) — python-пакет rob_box_sim.
ament_python_install_package(${PROJECT_NAME})

# worlds/local/ — миры из карты робота, в git и в образ не идут (карта дома).
install(
  DIRECTORY config launch scenarios worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "local" EXCLUDE
)

ament_package()
