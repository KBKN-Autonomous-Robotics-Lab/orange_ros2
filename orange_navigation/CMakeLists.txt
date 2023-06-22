cmake_minimum_required(VERSION 3.8)
project(orange_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
)

add_executable(waypoint_follower_client src/waypoint_follower_client.cpp)
ament_target_dependencies(waypoint_follower_client rclcpp rclcpp_action nav2_msgs geometry_msgs yaml-cpp)
target_link_libraries(waypoint_follower_client ${YAML_CPP_LIBRARIES})

# install
install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS
  waypoint_follower_client
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
