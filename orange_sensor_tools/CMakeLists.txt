cmake_minimum_required(VERSION 3.8)
project(orange_sensor_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0074 NEW)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(serial REQUIRED)

# add include directories
include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  laser_geometry
  tf2
  tf2_ros
  pcl_ros
  Eigen3
  PCL
)

# build
add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)
ament_target_dependencies(laserscan_multi_merger ${dependencies})
target_include_directories(laserscan_multi_merger PUBLIC ${EIGEN_INCLUDE_DIRS})
target_include_directories(laserscan_multi_merger PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(laserscan_multi_merger ${PCL_LIBRARIES})

install(TARGETS laserscan_multi_merger
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(led_node src/led_node.cpp)
ament_target_dependencies(led_node rclcpp std_msgs serial)

install(TARGETS led_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(wheel_imu_odom src/wheel_imu_odom.cpp)
ament_target_dependencies(wheel_imu_odom
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS wheel_imu_odom
  DESTINATION lib/${PROJECT_NAME})

# install
install(DIRECTORY config firmware include launch src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
