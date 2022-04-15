cmake_minimum_required(VERSION 3.5)
project(action_tutorials_cpp)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_library(action_server SHARED src/fibonacci_action_server.cpp)

target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(action_server PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

ament_target_dependencies(action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "tutorial_interfaces"
)

rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)


add_library(action_client SHARED src/fibonacci_action_client.cpp)

target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(action_client PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

ament_target_dependencies(action_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "tutorial_interfaces"
)

rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)


install(TARGETS
  action_server action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
