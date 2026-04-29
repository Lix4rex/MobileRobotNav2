cmake_minimum_required(VERSION 3.8)
project(car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(gt_node src/gt_node.cpp)
ament_target_dependencies(gt_node rclcpp nav_msgs sensor_msgs)

target_link_libraries(gt_node
  gz-transport13::core
)

add_executable(car_controller src/car_controller.cpp)
ament_target_dependencies(car_controller rclcpp geometry_msgs sensor_msgs)


# Monitors

add_executable(screen_monitor src/screen_monitor.cpp)
ament_target_dependencies(screen_monitor rclcpp std_msgs)

add_executable(match_monitor src/match_monitor.cpp)
ament_target_dependencies(match_monitor rclcpp std_msgs)

add_executable(calibration_monitor src/calibration_monitor.cpp)
ament_target_dependencies(calibration_monitor rclcpp std_msgs)

# Controllers

add_executable(oesophage_controller src/controllers/oesophage_controller.cpp)
ament_target_dependencies(oesophage_controller rclcpp std_msgs)

add_executable(stomach_controller src/controllers/stomach_controller.cpp)
ament_target_dependencies(stomach_controller rclcpp std_msgs)

add_executable(anus_controller src/controllers/anus_controller.cpp)
ament_target_dependencies(anus_controller rclcpp std_msgs)


install(TARGETS car_controller gt_node screen_monitor match_monitor calibration_monitor oesophage_controller stomach_controller anus_controller
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
