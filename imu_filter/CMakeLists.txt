cmake_minimum_required(VERSION 3.0.2)
project(imu_filter)

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

add_executable(imu_filter src/imu_filter.cpp)

target_link_libraries(imu_filter
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)