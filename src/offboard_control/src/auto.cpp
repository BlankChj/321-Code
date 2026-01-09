// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>

// mavros_msgs::State current_state;
// void state_cb(const mavros_msgs::State::ConstPtr &msg)
// {
//     current_state = *msg;
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "offb_node");
//     ros::NodeHandle nh;

//     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
//     ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
//     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
//     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

//     // the setpoint publishing rate MUST be faster than 2Hz
//     ros::Rate rate(20.0);

//     // wait for FCU connection
//     while (ros::ok() && !current_state.connected)
//     {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     std::vector<double> startPoint{0, 0, 0.5};
//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = 0;
//     pose.pose.position.y = 0;
//     pose.pose.position.z = 0.5;

//     // send a few setpoints before starting
//     for (int i = 100; ros::ok() && i > 0; --i)
//     {
//         local_pos_pub.publish(pose);
//         ros::spinOnce();
//         rate.sleep();
//     }

//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD";

//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;

//     ros::Time last_request = ros::Time::now();

//     while (ros::ok())
//     {
//         if (current_state.mode != "OFFBOARD" &&
//             (ros::Time::now() - last_request > ros::Duration(5.0)))
//         {
//             if (set_mode_client.call(offb_set_mode) &&
//                 offb_set_mode.response.mode_sent)
//             {
//                 ROS_INFO("Offboard enabled");
//             }
//             last_request = ros::Time::now();
//         }
//         else
//         {
//             if (!current_state.armed &&
//                 (ros::Time::now() - last_request > ros::Duration(5.0)))
//             {
//                 if (arming_client.call(arm_cmd) &&
//                     arm_cmd.response.success)
//                 {
//                     ROS_INFO("Vehicle armed");
//                 }
//                 last_request = ros::Time::now();
//             }
//         }

//         local_pos_pub.publish(pose);

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }



#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <vector>

// 当前无人机状态（如连接状态、当前模式）
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

// 当前位置（用于计算轨迹的相对坐标，可选）
geometry_msgs::PoseStamped local_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    // 1. 订阅状态和位置
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);

    // 2. 发布位置设定点 (Setpoint)
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // 3. 服务客户端：解锁和切模式
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 等待与飞控建立连接
    ros::Rate rate(20.0); // 发送频率 20Hz (必须 > 2Hz)
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection...");
    }

    std::vector<double> startPoint{0,0,0.5};
    // 初始化目标位置
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.5; 

    // 在切换到 Offboard 模式之前，必须先发送一些设定点
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now(); // 记录任务开始时间

    // --- 任务阶段标志位 ---
    bool is_hovering = false;
    bool is_tracking = false;
    bool is_landing = false;

    // 轨迹参数
    double radius = 1.0; // 半径1米
    double omega = 0.5;  // 角速度 rad/s

    ROS_INFO("Starting Mission...");

    bool pubFlag = true;
    std::vector<double> timeTable{0.0, 20.0, 50.0, 60.0};
    while (ros::ok())
    {
        double time_elapsed = (ros::Time::now() - start_time).toSec();

        // --- 逻辑 1: 自动解锁与切模 (仅在未完成时尝试) ---
        // 注意：实物实验建议手动遥控器切Offboard模式，代码仅作辅助
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)) && time_elapsed < timeTable[1])
        {
            ROS_INFO("Offboard enabled Starting...");
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)) && time_elapsed < timeTable[1])
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // --- 逻辑 2: 任务控制状态机 ---


        if (current_state.mode == "OFFBOARD" && current_state.armed)
        {
            // 阶段 1: 悬停 
            if (time_elapsed < timeTable[1])
            {
                pose.pose.position.x = startPoint[0];
                pose.pose.position.y = startPoint[1];
                pose.pose.position.z = startPoint[2];
                if (!is_hovering){
                    ROS_INFO("Task: Hovering");
                    is_hovering = true;
                }
            }
            // 阶段 2: 轨迹跟踪 
            else if (time_elapsed >= timeTable[1] && time_elapsed < timeTable[2])
            {
                is_hovering = false;
                if (!is_tracking)
                {
                    ROS_INFO("Task: Tracking Circle Trajectory");
                    is_tracking = true;
                }
                // 画圆逻辑 (基于时间生成坐标)
                double t_track = time_elapsed - timeTable[1];
                pose.pose.position.x = startPoint[0] + radius * sin(omega * t_track);
                pose.pose.position.y = startPoint[1] + radius * cos(omega * t_track);
                pose.pose.position.z = startPoint[2];
            }
            // 阶段 3: 降落 
            else if (time_elapsed >= timeTable[2] && time_elapsed < timeTable[3])
            {
                is_tracking = false;
                if (!is_landing)
                {
                    ROS_INFO("Task: Landing Mode Triggered");
                    is_landing = true;
                    pose.pose.position.x = startPoint[0];
                    pose.pose.position.y = startPoint[1];
                    pose.pose.position.z = 0.05;
                }
            }
            else if (time_elapsed >= timeTable[3])
            {
                pubFlag = false;

                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
            }
        }

        if (pubFlag){
            local_pos_pub.publish(pose);
        }
            ros::spinOnce();
            rate.sleep();
    }
    return 0;
}