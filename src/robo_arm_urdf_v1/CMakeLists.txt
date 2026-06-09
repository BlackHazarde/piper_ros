cmake_minimum_required(VERSION 3.8)
project(robo_arm_urdf_v1)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes rviz config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()