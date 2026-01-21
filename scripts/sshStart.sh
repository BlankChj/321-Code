#!/bin/bash
irisName=${1:-"0"}

if [ "$irisName" = "4" ]; then
    ip="192.168.31.163"
elif [ "$irisName" = "5" ]; then
    ip="192.168.31.163"
elif [ "$irisName" = "7" ]; then
    ip="192.168.31.36"
fi

PASSWORD="ss278551"
HOST="iris-$irisName@$ip"

echo "连接远程主机：$HOST"
echo "连接远程主机并执行命令..."
echo "cd ~/scripts; source auto_iris_launch.sh iris$irisName"

rm -f /tmp/known_hosts_temp

# 使用 sshpass 执行命令
sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
    "$HOST" -p 22 -Y "export LC_ALL=C; cd ~/scripts; source auto_iris_launch.sh iris$irisName; echo '命令执行完成'"
    

    # source auto_iris_launch.sh iris$irisName
# sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
#     "$HOST" -p 22 -Y "cd ~/scripts; source iris_launch.sh iris4; echo '命令执行完成'; exec bash "