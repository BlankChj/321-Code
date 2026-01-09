#!/bin/bash

# 关闭特定标题的终端窗口
titles="vrpn_launch|mavros_launch|topic_relay|udp_pkg|control_pkg"

# 检查是否安装 wmctrl
if ! command -v wmctrl &> /dev/null; then
    echo "wmctrl 未安装，请先安装它：sudo apt install wmctrl"
    exit 1
fi

# 查找并关闭匹配标题的终端窗口
for title in $(echo $titles | tr "|" "\n"); do
    wmctrl -l | grep "$title" | awk '{print $1}' | while read -r window_id; do
        wmctrl -ic "$window_id"
        echo "已关闭标题为 $title 的终端窗口 (窗口 ID: $window_id)"
    done
done

# #!/bin/bash

# # 查找并关闭与 auto_iris_launch.sh 脚本相关的所有 gnome-terminal 进程
# echo "正在关闭所有相关终端..."

# # 获取所有运行的 gnome-terminal 进程 ID
# pids=$(ps aux | grep -E "vrpn_launch|mavros_launch|topic_relay|udp_pkg|control_pkg" | grep -v grep | awk '{print $2}')

# # 遍历并杀死每个进程
# if [ -n "$pids" ]; then
#     echo "找到以下进程：$pids"
#     kill -9 $pids
#     echo "所有相关终端已关闭。"
# else
#     echo "未找到相关进程。"
# fi