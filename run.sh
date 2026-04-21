#!/bin/bash
# source /devel/setup.bash

echo "启动 PX4 ..."
gnome-terminal --tab --title="PX4" -- bash -c "roslaunch px4 fast_tests.launch; exec bash"
sleep 6

echo "启动 FastLIO MAVROS Bridge..."
gnome-terminal --tab --title="Fastlio Bridge" -- bash -c "rosrun fastlio_mavros_bridge fastlio_bridge; exec bash"
sleep 3

echo "启动 Fast_LIO ..."
gnome-terminal --tab --title="FastLIO Mapping" -- bash -c "roslaunch fast_lio mapping_mid360.launch; exec bash"
sleep 3

echo "启动 Ego Mavros Bridge ..."
gnome-terminal --tab --title="Ego Bridge" -- bash -c "rosrun fastlio_mavros_bridge ego_mavros_bridge; exec bash"
sleep 3

echo "启动 Diff_Planner..."
gnome-terminal --tab --title="Diff Planner" -- bash -c "roslaunch diff_planner run_exp_single_lio.launch; exec bash"
sleep 3

echo "启动 Drone_FSM Mission ..."
gnome-terminal --tab --title="State Machine" -- bash -c "rosrun drone_fsm drone_fsm_node; exec bash"
sleep 5

echo "启动 yolo节点 ..."
gnome-terminal --tab --title="yoloframe" -- bash -c "roslaunch frame_detection transform_gate_w.launch; exec bash"

sleep 3

gnome-terminal --tab --title="yoloaruco" -- bash -c "roslaunch aruco_pnp_vision transform_aruco_w.launch; exec bash"

sleep 3

gnome-terminal --tab --title="yoloballoon" -- bash -c "roslaunch balloon_vision transform_balloon_w.launch; exec bash"

sleep 3

gnome-terminal --tab --title="car1" -- bash -c "roslaunch armbot_nav armbot_offline_nav.launch; exec bash"

sleep 3

gnome-terminal --tab --title="car2" -- bash -c "rosrun armbot_nav auto_point.py; exec bash"


echo "所有节点均已启动完毕"
