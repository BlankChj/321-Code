#!/bin/bash

OptiIp=${1:-"192.168.31.136"}

gnome-terminal --tab --title="tab1" -- bash -c "echo test1 start; roscore"
gnome-terminal	--tab --title="tab2" -- bash -c " echo '$OptiIp'; exec bash"
