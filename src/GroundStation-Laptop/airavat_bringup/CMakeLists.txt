cmake_minimum_required(VERSION 3.8)
project(airavat_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
