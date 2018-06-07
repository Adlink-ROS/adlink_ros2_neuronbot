cmake_minimum_required(VERSION 3.5)
project(adlink_ros2_neuronbot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rcutils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(adlink_ros2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# following_legs
add_executable(following_legs src/following_legs.cpp)
ament_target_dependencies(following_legs 
rclcpp 
adlink_ros2_msgs 
geometry_msgs 
tf2
tf2_ros
tf2_geometry_msgs)


install(TARGETS
  following_legs
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
