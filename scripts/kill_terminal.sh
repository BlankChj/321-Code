#!/bin/bash

# 查找并关闭与 auto_iris_launch.sh 脚本相关的所有 gnome-terminal 进程
echo "正在关闭所有相关终端..."

# 获取所有运行的 gnome-terminal 进程 ID
pids=$(ps aux | grep -E "vrpn_launch|mavros_launch|topic_relay|udp_pkg|control_pkg" | grep -v grep | awk '{print $2}')

# 遍历并杀死每个进程
if [ -n "$pids" ]; then
    echo "找到以下进程：$pids"
    kill -9 $pids
    echo "所有相关终端已关闭。"
else
    echo "未找到相关进程。"
fi