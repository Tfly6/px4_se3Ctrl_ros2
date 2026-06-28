cmake_minimum_required(VERSION 3.10)
project(se3_hopf)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(se3_hopf_node
  src/se3_hopf_node.cpp
  src/se3_ctrl.cpp
)

target_include_directories(se3_hopf_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(se3_hopf_node
  geometry_msgs
  px4_msgs
  rclcpp
  std_msgs
  std_srvs
  trajectory_msgs
)

install(TARGETS se3_hopf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
