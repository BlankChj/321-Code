#!/bin/bash
irisName=${1:-"iris0"}

PASSWORD="ss278551"
HOST="$irisName@192.168.31.163"

echo "连接远程主机并执行命令..."

# 使用 sshpass 执行命令
sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
    "$HOST" -p 22 -Y "export LC_ALL=C; cd ~/scripts; source iris_launch.sh $irisName; echo '命令执行完成'"
    
# sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
#     "$HOST" -p 22 -Y "cd ~/scripts; source iris_launch.sh iris4; echo '命令执行完成'; exec bash "