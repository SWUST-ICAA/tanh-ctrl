cmake_minimum_required(VERSION 3.14)

project(tanh_ctrl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlatTrajectoryReference.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

add_library(tanh_ctrl_common
  src/common.cpp
)

target_link_libraries(tanh_ctrl_common Eigen3::Eigen)
target_link_libraries(tanh_ctrl_common Sophus::Sophus)
target_include_directories(tanh_ctrl_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(tanh_ctrl_px4_bridge
  src/px4_bridge.cpp
)

target_link_libraries(tanh_ctrl_px4_bridge
  Eigen3::Eigen
)
target_include_directories(tanh_ctrl_px4_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tanh_ctrl_px4_bridge
  px4_msgs
)

add_library(tanh_ctrl_blocks
  src/tanh_blocks.cpp
)

target_link_libraries(tanh_ctrl_blocks Eigen3::Eigen)
target_include_directories(tanh_ctrl_blocks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(tanh_ctrl_core
  src/tanh_controller.cpp
)

target_link_libraries(tanh_ctrl_core
  Eigen3::Eigen
  tanh_ctrl_blocks
  tanh_ctrl_common
)
target_include_directories(tanh_ctrl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(tanh_ctrl_node_lib
  src/tanh_node.cpp
)

target_link_libraries(tanh_ctrl_node_lib
  tanh_ctrl_px4_bridge
  tanh_ctrl_core
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(tanh_ctrl_node_lib "${cpp_typesupport_target}")
ament_target_dependencies(tanh_ctrl_node_lib
  geometry_msgs
  rclcpp
  px4_msgs
  std_msgs
)

add_executable(tanh_ctrl_node
  src/tanh_node_main.cpp
)

target_link_libraries(tanh_ctrl_node tanh_ctrl_node_lib)
ament_target_dependencies(tanh_ctrl_node
  rclcpp
)
target_include_directories(tanh_ctrl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  tanh_ctrl_common
  tanh_ctrl_px4_bridge
  tanh_ctrl_blocks
  tanh_ctrl_core
  tanh_ctrl_node_lib
  tanh_ctrl_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
