#!/bin/bash

pkgRoot=${1:-"catkin_ws"}

gnome-terminal	--tab --title="gc_data_plot" -- bash -c "cd ~/$pkgRoot ;source ~/$pkgRoot/devel/setup.sh; cd ~/$pkgRoot/src/udp_pkg/scripts ; python3 udp_attacker.py ; exec bash -i"
