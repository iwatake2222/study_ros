cmake_minimum_required(VERSION 3.5)
project(learning_tf2_cpp)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${tf2_geometry_msgs_INCLUDE_DIRS}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)


add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
  static_turtle_tf2_broadcaster
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  turtlesim
)

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
  turtle_tf2_broadcaster
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  turtlesim
)

add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
ament_target_dependencies(
  turtle_tf2_listener
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  turtlesim
)

add_executable(fixed_frame_tf2_broadcaster src/fixed_frame_tf2_broadcaster.cpp)
ament_target_dependencies(
  fixed_frame_tf2_broadcaster
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  turtlesim
)

add_executable(dynamic_frame_tf2_broadcaster src/dynamic_frame_tf2_broadcaster.cpp)
ament_target_dependencies(
  dynamic_frame_tf2_broadcaster
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  turtlesim
)

add_executable(turtle_tf2_listener_debug src/turtle_tf2_listener_debug.cpp)
ament_target_dependencies(
  turtle_tf2_listener_debug
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  turtlesim
)

add_executable(turtle_tf2_message_filter src/turtle_tf2_message_filter.cpp)
ament_target_dependencies(
  turtle_tf2_message_filter
  geometry_msgs
  message_filters
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)


install(TARGETS
  static_turtle_tf2_broadcaster turtle_tf2_broadcaster turtle_tf2_listener fixed_frame_tf2_broadcaster dynamic_frame_tf2_broadcaster turtle_tf2_listener_debug turtle_tf2_message_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
 

ament_package()
