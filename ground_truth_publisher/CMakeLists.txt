cmake_minimum_required(VERSION 3.0.2)
project(ground_truth_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ground_truth_publisher src/ground_truth_publisher.cpp)
target_link_libraries(ground_truth_publisher ${catkin_LIBRARIES})