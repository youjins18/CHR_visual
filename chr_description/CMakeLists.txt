cmake_minimum_required(VERSION 3.8)
project(chr_description)

find_package(ament_cmake REQUIRED)
install(DIRECTORY mujoco meshes urdf DESTINATION share/${PROJECT_NAME})

ament_package()
