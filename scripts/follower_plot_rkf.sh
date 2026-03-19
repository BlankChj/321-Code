#!/bin/bash

pkgRoot=${1:-"catkin_ws"}

gnome-terminal	--tab --title="follower_plot_rkf" -- bash -c "cd ~/$pkgRoot ;source ~/$pkgRoot/devel/setup.sh;cd ~/$pkgRoot/src/plot/scripts; python3 plot_status.py; exec bash -i"
