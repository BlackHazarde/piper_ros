cmake_minimum_required(VERSION 3.8)
project(catch_moveit_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(catch_moveit_planner src/catch_moveit_planner.cpp)
target_compile_features(catch_moveit_planner PUBLIC cxx_std_17)
ament_target_dependencies(
  catch_moveit_planner
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)

install(TARGETS catch_moveit_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

