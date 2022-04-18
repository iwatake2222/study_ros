cmake_minimum_required(VERSION 3.5)
project(my_package)

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
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_interface REQUIRED)

### first_node ###
add_library(first_node SHARED src/first_node.cpp)

target_include_directories(first_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(first_node PRIVATE "MY_PACKAGE_BUILDING_DLL")

ament_target_dependencies(first_node
  "rclcpp"
  "rclcpp_components"
  "rclcpp_action"
  "std_msgs"
  "my_interface"
)

rclcpp_components_register_node(first_node PLUGIN "my_package::FirstNode" EXECUTABLE first)
# rclcpp_components_register_nodes(first_node "my_package::FirstNode")


### second ###
add_library(second_node SHARED src/second_node.cpp)

target_include_directories(second_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(second_node PRIVATE "MY_PACKAGE_BUILDING_DLL")

ament_target_dependencies(second_node
  "rclcpp"
  "rclcpp_components"
  "rclcpp_action"
  "std_msgs"
  "my_interface"
)

rclcpp_components_register_node(second_node PLUGIN "my_package::SecondNode" EXECUTABLE second)
# rclcpp_components_register_nodes(second_node "my_package::SecondNode")


install(TARGETS
  first_node second_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

