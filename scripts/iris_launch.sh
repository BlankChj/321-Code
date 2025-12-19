#!/bin/bash

irisName=${1:-"iris0"}
pkgRoot=${2:-"catkin_ws"}
optiIp=${3:-"192.168.31.136"}

gnome-terminal --tab --title="vrpn_launch" -- bash -c "source ~/$pkgRoot/devel/setup.sh;roslaunch vrpn_client_ros sample.launch server:=$optiIp"
sleep 10

gnome-terminal	--tab --title="mavros_launch" -- bash -c "roslaunch mavros px4.launch"
sleep 15

gnome-terminal	--tab --title="topic_relay" -- bash -c "rosrun topic_tools relay /vrpn_client_node/$irisName/pose /mavros/vision_pose/pose"
#gnome-terminal	--tab --title="topic_relay" -- bash -c "echo topic_tools relay /vrpn_client_node/$irisName/pose /mavros/vision_pose/pose; exec bash"
sleep 2

gnome-terminal	--tab --title="visionPose_display" -- bash -c "rostopic echo /mavros/vision_pose/pose"
sleep 2

gnome-terminal	--tab --title="localPose_display" -- bash -c " rostopic echo /mavros/local_position/pose"
sleep 2

gnome-terminal	--tab --title="udp_pkg" -- bash -c "cd ~/$pkgRoot/src/udp_pkg/scripts;echo Please Run the Udp Pkg; exec bash"

gnome-terminal	--tab --title="control_pkg" -- bash -c "cd ~/$pkgRoot ;source ~/$pkgRoot/devel/setup.sh;echo Please Run the Control Pkg; exec bash"
