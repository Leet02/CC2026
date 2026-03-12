#!/bin/bash
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source ${PROJECT_DIR}/devel/setup.bash

drone_type="x280"
drone_id=1
cx=321.04638671875  # 您的相机内参
cy=243.44969177246094
fx=387.229248046875
fy=387.229248046875
max_vel=2.0
max_acc=3.0
ego_swarm_flage=false  # 单机
flight_type=1  # 手动2D Nav Goal

rosparam set clean_localmap true

roslaunch ego_planner swarm_all_in_one.launch \
  drone_type:=$drone_type drone_id:=$drone_id cx:=$cx cy:=$cy fx:=$fx fy:=$fy \
  flight_type:=$flight_type max_vel:=$max_vel max_acc:=$max_acc ego_swarm_flage:=$ego_swarm_flage \
  odometry_topic:="/fastlio_odom" cloud_topic:="/fastlio_pointcloud" \
  camera_pose_topic:="/mavros/local_position/pose" depth_topic:="/depth/image_raw"