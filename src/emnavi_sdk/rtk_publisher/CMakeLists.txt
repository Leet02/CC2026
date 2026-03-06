cmake_minimum_required(VERSION 3.0.2)
project(rtk_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  serial
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rtk_publisher src/rtk_to_mavros.cpp)
target_link_libraries(rtk_publisher ${catkin_LIBRARIES})