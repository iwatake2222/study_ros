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
find_package(sensor_msgs REQUIRED)

add_library(pointcloud2_talker SHARED src/pointcloud2_talker.cpp)
target_include_directories(pointcloud2_talker PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# target_compile_definitions(pointcloud2_talker PRIVATE "MY_PACKAGE_BUILDING_DLL")
ament_target_dependencies(pointcloud2_talker
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs"
)
rclcpp_components_register_node(pointcloud2_talker PLUGIN "my_package::Pointcloud2Talker" EXECUTABLE pointcloud2_talker_exe)

install(TARGETS
  pointcloud2_talker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS
#   manual_composition
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
