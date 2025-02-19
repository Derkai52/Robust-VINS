cmake_minimum_required(VERSION 3.0.2)
project(global_smooth_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(global_smooth_fusion src/global_smooth_fusion.cpp)

target_link_libraries(global_smooth_fusion
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)