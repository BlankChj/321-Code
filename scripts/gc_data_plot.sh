#!/bin/bash

pkgRoot=${1:-"catkin_ws"}

gnome-terminal	--tab --title="gc_data_plot" -- bash -c "cd ~/$pkgRoot ;source ~/$pkgRoot/devel/setup.sh; cd ~/$pkgRoot/src/udp_pkg/src ; python3  3d_trajectory.py  ; exec bash -i"
