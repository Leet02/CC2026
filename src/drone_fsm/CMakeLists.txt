cmake_minimum_required(VERSION 3.0.2)
project(drone_fsm)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  tf
  rosconsole  # 添加：解决 ros::console 未定义
  quadrotor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_fsm
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs tf rosconsole
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(drone_fsm_node src/drone_fsm_node.cpp)
target_link_libraries(drone_fsm_node ${catkin_LIBRARIES})  # 关键添加：链接 ROS 库