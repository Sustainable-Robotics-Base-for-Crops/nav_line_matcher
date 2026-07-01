cmake_minimum_required(VERSION 3.8)
project(nav_line_matcher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  nav2_util
  nav_util
  std_msgs
  nav_msgs
  geometry_msgs
  lifecycle_msgs
  nav_interfaces
  angles)

find_package(ament_cmake REQUIRED)
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/line_matcher_server.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies})
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::LineMatcherServer"
  EXECUTABLE ${PROJECT_NAME}_node)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
