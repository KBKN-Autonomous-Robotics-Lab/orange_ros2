cmake_minimum_required(VERSION 3.8)
project(orange_sensor_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

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

# install
install(DIRECTORY src launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()