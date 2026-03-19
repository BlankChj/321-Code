无人集群安全状态估计操作指南：

//* 无人节点启动 5号机为主机 4,7号机为从机 *//

//主机
source leader_iris5_launch.sh
最后一个端口输入 rosrun offboard_control offboard进行飞行启动

//从机
source follower_iris4_launch.sh
最后一个端口输入 rosrun follower_control follower_control 1.5 0 0进行飞行启动
随后进行数据保存 source follower_data_save.sh
保存完后绘图  source follower_plot_rkf.sh

source follower_iris7_launch.sh
最后一个端口输入 rosrun follower_control follower_control -1.5 0 0进行飞行启动
随后进行数据保存 source follower_data_save.sh
保存完后绘图  source follower_plot_rkf.sh

//地面站主机
从机启动后
地面站主机开始攻击
source gc_attack.sh
地面站主机接受信息
source gc_data_receive.sh
接受完信息后
source gc_data_plot.sh
