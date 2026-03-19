#!/bin/bash

pkgRoot=${1:-"catkin_ws"}

gnome-terminal	--tab --title="follower_data_save" -- bash -c "cd ~/$pkgRoot ;source ~/$pkgRoot/devel/setup.sh;rosrun data_save data_save; exec bash -i"
