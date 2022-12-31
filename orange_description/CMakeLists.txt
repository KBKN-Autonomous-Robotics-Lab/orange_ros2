cmake_minimum_required(VERSION 3.8)
project(orange_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(velodyne_gazebo_plugins REQUIRED)

# install
install(DIRECTORY meshes xacro
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()