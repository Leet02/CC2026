cmake_minimum_required(VERSION 3.0.2)
project(fastlio_mavros_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  quadrotor_msgs
  tf2
)


catkin_package(
   CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf2 quadrotor_msgs
#  INCLUDE_DIRS include
#  LIBRARIES fastlio_mavros_bridge
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fastlio_bridge src/fastlio_bridge.cpp)
add_executable(ego_mavros_bridge src/ego_mavros_bridge.cpp)
add_executable(nav2pos src/nav2pos.cpp)

target_link_libraries(fastlio_bridge
  ${catkin_LIBRARIES}
)

target_link_libraries(ego_mavros_bridge ${catkin_LIBRARIES})

target_link_libraries(nav2pos
  ${catkin_LIBRARIES}
)

add_dependencies(fastlio_bridge
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


add_dependencies(ego_mavros_bridge
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(nav2pos 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}  # 确保 MyMsg.h 先生成
  ${catkin_EXPORTED_TARGETS}
)