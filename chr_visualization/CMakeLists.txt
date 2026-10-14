cmake_minimum_required(VERSION 3.8)
project(chr_visualization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(chr_rviz src/chr_rviz.cpp)
target_compile_features(chr_rviz PUBLIC cxx_std_17)
ament_target_dependencies(chr_rviz
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
  tf2 tf2_ros visualization_msgs)

install(TARGETS chr_rviz DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
