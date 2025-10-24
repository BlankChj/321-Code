#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <udp_pkg/PositionVelocityAccel.h>
#include <kalman_filter/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <nav_msgs/Path.h>
#include <termios.h>
#include <iostream>
#include <string>
#include <pthread.h>
#include <unistd.h>


using namespace std;
#define PI 3.1415926

struct Position
{
    double x;
    double y;
    double z;
};


// Re_Location is the relative location to iris_0 
//     iris_0    iris_1   iris_2
// x  ......     ......   ......
// y  ......     ......   ......
// z  ......     ......   ......
// double Re_Location[3][3] = {
//     {0,3,0},
//     {0,0,3},
//     {0,0,0}
// };
double Re_Location[3][2] = {
    {0, 3},
    {0, 0},
    {0, 0}};

// k is the pid Parameter 
//     P    D   I
// kx  ..  ..  ..
// ky  ..  ..  ..
// kz  ..  ..  ..
double k[3][3] = {
    {1,0,0},
    {1,0,0},
    {1,0,0},
};

geometry_msgs::PoseStamped desired_pose;
geometry_msgs::PoseStamped desired_pose1;
geometry_msgs::PoseStamped desired_pose2;
geometry_msgs::PoseStamped desired_pose3;
void leader_pose_cb_udp(const udp_pkg::PositionVelocityAccel::ConstPtr &msg)
{
    udp_pkg::PositionVelocityAccel pose = *msg;
    desired_pose1.header.stamp = ros::Time::now();
    desired_pose1.header.seq = 0;
    desired_pose1.pose.position.x = pose.x_pos + Re_Location[0][1];
    desired_pose1.pose.position.y = pose.y_pos + Re_Location[1][1];
    desired_pose1.pose.position.z = pose.z_pos + Re_Location[2][1];
}

void leader_pose_cb_kf(const kalman_filter::Vector3Stamped::ConstPtr &msg)
{
    kalman_filter::Vector3Stamped pose = *msg;
    desired_pose2.header.stamp =ros::Time::now();
    desired_pose2.header.seq = 0;
    desired_pose2.pose.position.x = pose.x + Re_Location[0][1];
    desired_pose2.pose.position.y = pose.y + Re_Location[1][1];
    desired_pose2.pose.position.z = pose.z + Re_Location[2][1];
}

void leader_pose_cb_rkf(const kalman_filter::Vector3Stamped::ConstPtr &msg)
{
    kalman_filter::Vector3Stamped pose = *msg;
    desired_pose3.header.stamp = ros::Time::now();
    desired_pose3.header.seq = 0;
    desired_pose3.pose.position.x = pose.x + Re_Location[0][1];
    desired_pose3.pose.position.y = pose.y + Re_Location[1][1];
    desired_pose3.pose.position.z = pose.z + Re_Location[2][1];
}

nav_msgs::Path nav_path;
geometry_msgs::PoseStamped follower_1_pose;
void follower_1_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    geometry_msgs::PoseStamped pose = *msg;
    follower_1_pose.header.stamp =ros::Time::now();
    follower_1_pose.header.seq = 0;
    follower_1_pose.pose.position.x = pose.pose.position.x;
    follower_1_pose.pose.position.y = pose.pose.position.y;
    follower_1_pose.pose.position.z = pose.pose.position.z;
    follower_1_pose.pose.orientation.x = pose.pose.orientation.x;
    follower_1_pose.pose.orientation.y = pose.pose.orientation.y;
    follower_1_pose.pose.orientation.z = pose.pose.orientation.z;
    follower_1_pose.pose.orientation.w = pose.pose.orientation.w;
    nav_path.poses.push_back(follower_1_pose);
}

nav_msgs::Path track_path;
geometry_msgs::PoseStamped track_pose;
void track_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    track_pose = *msg;
    track_path.poses.push_back(track_pose);
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

char mode_key = '1';
char get_key = '1';
void *scanKeyboard(void *args)
{
    while (true)
    {
        int in;
        struct termios new_settings;
        struct termios stored_settings;
        tcgetattr(0, &stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= (~ICANON);
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(0, &stored_settings);
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0, TCSANOW, &new_settings);
        in = getchar();
        tcsetattr(0, TCSANOW, &stored_settings);
        get_key = in;
        if (get_key == 'x')
            pthread_exit(NULL);
    }
}

int main(int argc, char **argv)
{
    pthread_t thread1;
    if (pthread_create(&thread1, NULL, scanKeyboard, NULL))
    {
        cerr << "can not creat pthread" << endl;
    }
    else
        cout << "pthread creat success, wait 3s" << endl;
    sleep(3);
    cout << "ROS program start" << endl << endl;

    ros::init(argc, argv, "follower_1_node");
    ros::NodeHandle nh;
    
    nav_path.header.frame_id= "world";
    nav_path.header.stamp = ros::Time::now();
    track_path.header.frame_id= "world";
    track_path.header.stamp = ros::Time::now();
    
    ros::Subscriber follower_1_state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    
    ros::Publisher follower_1_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    	    
    ros::ServiceClient follower_1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient follower_1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    
    // ros::Subscriber follower_1_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //         ("/follower_1/mavros/vision_pose/pose", 10, pose_cb);  
    // 这里需要重新确定一下位置信息，local可能是以上电位置为原点
    ros::Subscriber follower_1_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, follower_1_pose_cb);

    ros::Subscriber leader_pose_sub_udp = nh.subscribe<udp_pkg::PositionVelocityAccel>
            ("/leader/information", 10, leader_pose_cb_udp);
    ros::Subscriber leader_pose_sub_kf = nh.subscribe<kalman_filter::Vector3Stamped>
            ("/leader/kf/pos", 10, leader_pose_cb_kf);
    ros::Subscriber leader_pose_sub_rkf = nh.subscribe<kalman_filter::Vector3Stamped>
            ("/leader/rkf/pos", 10, leader_pose_cb_rkf);

    ros::Publisher follower_1_path_pub = nh.advertise<nav_msgs::Path>("/follower_1/path_pub", 10);
    ros::Publisher follower_1_track_pub = nh.advertise<nav_msgs::Path>("/follower_1/track_pub", 10);

    ros::Publisher follower_1_local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    
    // ros::Subscriber follower_1_track_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //         ("/vrpn_client_node/zx_chj_ss/pose", 10, track_cb);
    //实飞时取消注释，改成正确的话题


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }



    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0.1;
    // cmd_vel.angular.x = 0;
    // cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        follower_1_local_pos_pub.publish(desired_pose1);
        // local_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    Position last_pos_error = {
        desired_pose1.pose.position.x - follower_1_pose.pose.position.x,
        desired_pose1.pose.position.y - follower_1_pose.pose.position.y,
        desired_pose1.pose.position.z - follower_1_pose.pose.position.z
    };
    Position now_pos_error = {
        desired_pose1.pose.position.x - follower_1_pose.pose.position.x,
        desired_pose1.pose.position.y - follower_1_pose.pose.position.y,
        desired_pose1.pose.position.z - follower_1_pose.pose.position.z
    };

    Position d_pos_error ={
        0,0,0
    };

    Position i_pos_error ={
        0,0,0
    };
    
    ros::Time last_time = ros::Time::now();
    ros::Time now_time = ros::Time::now();
    double time_gap;
    long long count = 0;
    char last_key;
    while(ros::ok()){
        count += 1;

        if (get_key >= '1' && get_key <= '4')
        {
            mode_key = get_key;
            get_key = '0';
            last_key = mode_key;
        }

        if (mode_key == '1') {
            desired_pose = desired_pose1;
            if (last_key != mode_key){
                std::cout << "<<<<<<<< Raw Data >>>>>>>>" << std::endl;
                last_key = mode_key;
            }
        }
        if (mode_key == '2')
        {
            desired_pose = desired_pose2;
            if (last_key != mode_key){
                std::cout << "<<<<<<<< KF Filtered Data >>>>>>>>" << std::endl;
                last_key = mode_key;
            }
        }
        if (mode_key == '3')
        {
            desired_pose = desired_pose3;
            if (last_key != mode_key){
                std::cout << "<<<<<<<< RKF Filtered Data >>>>>>>>" << std::endl;
                last_key = mode_key;
            }
        }
        if (mode_key == '4')
        {
            desired_pose.pose.position.x = 0.91;
            desired_pose.pose.position.x = 0.66;
            desired_pose.pose.position.x = 0.05;
            if (last_key != mode_key){
                std::cout << "<<<<<<<< Security Landing!!! >>>>>>>>" << std::endl;
                last_key = mode_key;
            }
        }
        last_pos_error = now_pos_error;
        ros::spinOnce();
        now_pos_error.x = desired_pose.pose.position.x - follower_1_pose.pose.position.x;
        now_pos_error.y = desired_pose.pose.position.y - follower_1_pose.pose.position.y;
        now_pos_error.z = desired_pose.pose.position.z - follower_1_pose.pose.position.z;


        now_time = ros::Time::now();
        time_gap = (now_time - last_time).toSec();
        last_time = now_time;

        d_pos_error.x = (now_pos_error.x - last_pos_error.x) / time_gap;
        d_pos_error.y = (now_pos_error.y - last_pos_error.y) / time_gap;
        d_pos_error.z = (now_pos_error.z - last_pos_error.z) / time_gap;

        i_pos_error.x += now_pos_error.x * time_gap;
        i_pos_error.y += now_pos_error.y * time_gap;
        i_pos_error.z += now_pos_error.z * time_gap;

        // if ( current_state.mode != "OFFBOARD"){
        //     if( follower_1_set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        // }

        // if ( !current_state.armed){
        //     if( follower_1_arming_client.call(arm_cmd) &&
        //         arm_cmd.response.success){
        //         ROS_INFO("Vehicle armed");
        //     }
        // }

        cmd_vel.linear.x = k[0][0]* now_pos_error.x + k[0][1] * d_pos_error.x + k[0][2] * i_pos_error.x;
        cmd_vel.linear.y = k[1][0]* now_pos_error.y + k[1][1] * d_pos_error.y + k[1][2] * i_pos_error.y;
        cmd_vel.linear.z = k[2][0]* now_pos_error.z + k[2][1] * d_pos_error.z + k[2][2] * i_pos_error.z;

        // cmd_vel.angular.x = 0;
        // cmd_vel.angular.y = 0;
        // cmd_vel.angular.z = 0;

        double max_vel_norm = 1;
        double norm_x = pow(cmd_vel.linear.x, 2);
        double norm_y = pow(cmd_vel.linear.y, 2);
        double norm_z = pow(cmd_vel.linear.z, 2);
        double norm = pow(norm_x + norm_y + norm_z, 0.5);
        if (norm > max_vel_norm) {
            cmd_vel.linear.x = max_vel_norm * cmd_vel.linear.x / norm;
            cmd_vel.linear.y = max_vel_norm * cmd_vel.linear.y / norm;
            cmd_vel.linear.z = max_vel_norm * cmd_vel.linear.z / norm;
        }
        follower_1_local_vel_pub.publish(cmd_vel);
	

	    follower_1_path_pub.publish(nav_path);
        //local_pos_pub.publish(pose);
	
        rate.sleep();
    }

    return 0;
}
