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
find_package(nav2_msgs REQUIRED)
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

add_executable(calibration_monitor src/calibration_monitor.cpp)
ament_target_dependencies(calibration_monitor rclcpp std_msgs)


install(TARGETS car_controller gt_node screen_monitor calibration_monitor
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)


# Actions

find_package(custom_action_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)


# CLIENT
add_library(action_client SHARED
  src/match_monitor.cpp
)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_client
  PRIVATE "CAR_BUILDING_DLL"
)
ament_target_dependencies(action_client
  "std_msgs"
  "nav2_msgs"
  "custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)
rclcpp_components_register_node(action_client PLUGIN "car::MatchMonitor" EXECUTABLE match_monitor)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


add_library(action_server SHARED
  src/controllers/oesophage_action_server.cpp
  src/controllers/stomach_action_server.cpp
  src/controllers/anus_action_server.cpp
)
  
### OESOPHAGE SERVER ###
rclcpp_components_register_node(action_server PLUGIN "car::OesophageActionServer" EXECUTABLE oesophage_action_server)

### STOMACH SERVER ###
rclcpp_components_register_node(action_server PLUGIN "car::StomachActionServer" EXECUTABLE stomach_action_server)

### ANUS SERVER ###
rclcpp_components_register_node(action_server PLUGIN "car::AnusActionServer" EXECUTABLE anus_action_server)


target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_server
  PRIVATE "CAR_BUILDING_DLL")
ament_target_dependencies(action_server
  "custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)

install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
