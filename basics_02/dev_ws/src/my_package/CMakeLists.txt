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
find_package(std_msgs REQUIRED)

add_library(talker SHARED src/talker.cpp)
target_include_directories(talker PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(talker PRIVATE "MY_PACKAGE_BUILDING_DLL")
ament_target_dependencies(talker
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)
rclcpp_components_register_node(talker PLUGIN "my_package::Talker" EXECUTABLE talker_exe)

add_library(listener SHARED src/listener.cpp)
target_include_directories(listener PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(listener PRIVATE "MY_PACKAGE_BUILDING_DLL")
ament_target_dependencies(listener
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)
rclcpp_components_register_node(listener PLUGIN "my_package::Listener" EXECUTABLE listener_exe)

install(TARGETS
  talker listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

