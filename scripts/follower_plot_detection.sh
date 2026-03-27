#!/bin/bash

pkgRoot=${1:-"catkin_ws"}

gnome-terminal	--tab --title="follower_plot_detection" -- bash -c "cd ~/$pkgRoot ;source ~/$pkgRoot/devel/setup.sh;cd ~/$pkgRoot/src/plot/scripts; python3 detection_plot_static.py; exec bash -i"
