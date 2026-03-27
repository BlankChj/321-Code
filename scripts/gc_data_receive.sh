#!/bin/bash

pkgRoot=${1:-"attack_ws"}

gnome-terminal	--tab --title="gc_data_receive" -- bash -c "cd ~/$pkgRoot ;source ~/$pkgRoot/devel/setup.sh; cd ~/$pkgRoot/src/udp_pkg/scripts ; python3 csv_gen.py 3 ; exec bash -i"
