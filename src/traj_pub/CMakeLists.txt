cmake_minimum_required(VERSION 3.14)

project(traj_pub)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(flat_trajectory_msgs REQUIRED)

add_executable(lissajous_traj_pub
  src/lissajous_traj_pub.cpp
)

ament_target_dependencies(lissajous_traj_pub
  rclcpp
  std_msgs
  flat_trajectory_msgs
)

install(TARGETS
  lissajous_traj_pub
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
