cmake_minimum_required(VERSION 3.14)

project(tanh_ctrl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(flat_trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Sophus 1.24.6 QUIET)
if(NOT Sophus_FOUND)
  include(FetchContent)
  FetchContent_Declare(sophus_header_only
    GIT_REPOSITORY https://github.com/strasdat/Sophus.git
    GIT_TAG d0b7315a0d90fc6143defa54596a3a95d9fa10ec
    GIT_SHALLOW TRUE
  )
  FetchContent_GetProperties(sophus_header_only)
  if(NOT sophus_header_only_POPULATED)
    FetchContent_Populate(sophus_header_only)
  endif()

  add_library(sophus_header_only INTERFACE)
  add_library(Sophus::Sophus ALIAS sophus_header_only)
  target_include_directories(sophus_header_only INTERFACE
    ${sophus_header_only_SOURCE_DIR}
  )
  target_link_libraries(sophus_header_only INTERFACE Eigen3::Eigen)
endif()

add_executable(tanh_ctrl_node
  src/main.cpp
  src/node.cpp
  src/controller.cpp
)

target_link_libraries(tanh_ctrl_node
  Eigen3::Eigen
  Sophus::Sophus
)
ament_target_dependencies(tanh_ctrl_node
  rclcpp
  px4_msgs
  std_msgs
  flat_trajectory_msgs
)
target_include_directories(tanh_ctrl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  tanh_ctrl_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tanh_ctrl_param_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tanh_ctrl_param_gui
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
