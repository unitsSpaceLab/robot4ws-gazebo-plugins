cmake_minimum_required(VERSION 3.10)
project(robot4ws_gazebo_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot4ws_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ----------------------------------------------------------------------------
# Include paths
# ----------------------------------------------------------------------------
include_directories(
  include
)

# ----------------------------------------------------------------------------
# Build your plugins
# ----------------------------------------------------------------------------
add_library(JointsControllerPlugin SHARED
  src/JointsControllerPlugin.cc
)

add_library(JointsControllerPluginDynamixel SHARED
  src/JointsControllerPluginDynamixel.cc
)
set(COMMON_DEPENDENCIES
  gz-sim8
  ament_index_cpp
)

ament_target_dependencies(JointsControllerPlugin
  ${COMMON_DEPENDENCIES}
  gz-transport13
  gz-msgs10
)

target_link_libraries(JointsControllerPlugin
  yaml-cpp  # <--- Direct link to correct lib, yaml-cpp not handled correctly by ament_target_dependencies
)

ament_target_dependencies(JointsControllerPluginDynamixel
  ${COMMON_DEPENDENCIES}
  rclcpp
  std_msgs
  robot4ws_msgs
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
)

target_link_libraries(JointsControllerPluginDynamixel
  yaml-cpp  # <--- Direct link to correct lib, yaml-cpp not handled correctly by ament_target_dependencies
)

# ----------------------------------------------------------------------------
# Install
# ----------------------------------------------------------------------------
install(
  TARGETS JointsControllerPlugin
  LIBRARY DESTINATION lib
)

install(
  TARGETS JointsControllerPluginDynamixel
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# ----------------------------------------------------------------------------
# Export
# ----------------------------------------------------------------------------
ament_package()
