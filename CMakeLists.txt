cmake_minimum_required(VERSION 3.5)
project(duro_gps_driver)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
# add_compile_options(-std=c++17 -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

link_directories("/usr/local/lib/")
include_directories(
  include 
  "/usr/local/include"
)

set(ament_dependencies
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(duro_gps_driver_node 
  src/duro_gps_driver.cpp 
  src/utm.cpp 
  src/fake_orientation.cpp
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Heading.msg"
  DEPENDENCIES std_msgs
)

rosidl_target_interfaces(duro_gps_driver_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(duro_gps_driver_node sbp)
ament_target_dependencies(duro_gps_driver_node ${ament_dependencies})
ament_export_dependencies(rosidl_default_runtime)

## Install
## Mark executables and/or libraries for installation
install(TARGETS duro_gps_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

## Mark launch files for installation
install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()