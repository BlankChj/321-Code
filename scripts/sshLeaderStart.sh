#!/bin/bash
irisName=${1:-"0"}

ip4="192.168.31.163" ##leader
ip5="192.168.31.136"  ##follower1
ip7="192.168.31.36"  ##follower2


PASSWORD="ss278551"
HOST="iris-$irisName@$ip"

echo "连接远程主机：$HOST"
echo "连接远程主机并执行命令..."
echo "cd ~/scripts; source auto_iris_launch.sh iris$irisName"

# 使用 sshpass 执行命令
sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
    "$HOST" -p 22 -Y "export LC_ALL=C; cd ~/scripts; source auto_leader_launch.sh iris$irisName $ip4 $ip5 $ip7; echo '命令执行完成'"
    

    # source auto_iris_launch.sh iris$irisName
# sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
#     "$HOST" -p 22 -Y "cd ~/scripts; source iris_launch.sh iris4; echo '命令执行完成'; exec bash "