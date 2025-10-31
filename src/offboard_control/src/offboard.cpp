#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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

void print_pose(geometry_msgs::PoseStamped pose){
    cout << "current target point: x=" << pose.pose.position.x << " y=" << 
    pose.pose.position.y << " z=" << pose.pose.position.z << endl << endl;
}

void print_cmd(){
    cout << "1: point control" << endl;
    cout << "2: path control" << endl;
    cout << "3: land control" << endl;
    cout << "4: track control" << endl;
    cout << "w: x-direction increase 0.05" << endl;
    cout << "s: x-direction decrease 0.05" << endl;
    cout << "a: y-direction increase 0.05" << endl;
    cout << "d: y-direction decrease 0.05" << endl;
    cout << "q: z-direction increase 0.05" << endl;
    cout << "e: z-direction decrease 0.05" << endl;
    cout << "press x and then Ctrl+C: exit program" << endl << endl;
}

char mode_key = '1';
char get_key = '1';

void* scanKeyboard(void *args){
    while(true){
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
        if (get_key == 'x') pthread_exit(NULL);
    }
}


nav_msgs::Path nav_path;
geometry_msgs::PoseStamped path_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    geometry_msgs::PoseStamped pose = *msg;
    path_pose.header.stamp =ros::Time::now();
    path_pose.header.seq = 0;
    path_pose.pose.position.x = pose.pose.position.x;
    path_pose.pose.position.y = pose.pose.position.y;
    path_pose.pose.position.z = pose.pose.position.z;
    path_pose.pose.orientation.x = pose.pose.orientation.x;
    path_pose.pose.orientation.y = pose.pose.orientation.y;
    path_pose.pose.orientation.z = pose.pose.orientation.z;
    path_pose.pose.orientation.w = pose.pose.orientation.w;
    nav_path.poses.push_back(path_pose);
}

nav_msgs::Path track_path;
geometry_msgs::PoseStamped track_pose;
void track_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //geometry_msgs::PoseStamped pose = *msg;
    track_pose = *msg;
    /*
    track_pose.pose.position.x = pose.pose.position.x;
    track_pose.pose.position.y = pose.pose.position.y;
    track_pose.pose.position.z = pose.pose.position.z;
    */
    track_path.poses.push_back(track_pose);
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    pthread_t thread1;
    if(pthread_create(&thread1, NULL, scanKeyboard, NULL)){
        cerr << "can not creat pthread" << endl;
    }
    else cout << "pthread creat success, wait 3s" << endl;
    sleep(3);
    cout << "ROS program start" << endl << endl;
    double startPoint[3]{0,0,0.5};
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    nav_path.header.frame_id= "world";
    nav_path.header.stamp = ros::Time::now();
    track_path.header.frame_id= "world";
    track_path.header.stamp = ros::Time::now();
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    	    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 10, pose_cb);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path_pub", 10);
    ros::Publisher track_pub = nh.advertise<nav_msgs::Path>("/track_pub", 10);
    
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    
    ros::Subscriber track_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/zx_chj_ss/pose", 10, track_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = startPoint[0];  //0.8
    pose.pose.position.y = startPoint[1];  //-2.9
    pose.pose.position.z = startPoint[2];
    print_cmd();
    print_pose(pose);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0.1;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
	//local_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    long long count = 0;
    while(ros::ok()){
        count += 1;
    	/*
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

	if (count < 600) pose.pose.position.z = 1;
	else pose.pose.position.z = 0.3;
	*/
	
	/*
	cout << "---------------" << get_key << "---------------" << endl;
	if (get_key == 'a'){
	    cout << "++++" << get_key << "++++" << endl;
	    get_key = '0';
	}
	*/
	
	if (get_key >= '1' && get_key <= '4'){
	    mode_key = get_key;
	    get_key = '0';
	    if (mode_key == '1') {
	        print_cmd();
                print_pose(pose);
	    }
	    if (mode_key == '2') {
	        print_cmd();
	        cout << "start path control" << endl << endl;
	    }
	    if (mode_key == '3') cout << "start land control" << endl << endl;
	    if (mode_key == '4') cout << "start track control" << endl << endl;
	}
	
	if (mode_key == '2'){
	    pose.pose.position.x = startPoint[0] + 0.6*cos(count*PI/200);
	    pose.pose.position.y = startPoint[1] + 0.6*sin(count*PI/200);
	    pose.pose.position.z = startPoint[2];
	}
	
	if (mode_key == '3'){
	    pose.pose.position.x = startPoint[0];
	    pose.pose.position.y = startPoint[1];
	    pose.pose.position.z = 0.05;
	}
	
	if (mode_key == '4'){
	    pose.pose.position.x = track_pose.pose.position.x - 1.6;
	    pose.pose.position.y = track_pose.pose.position.y - 1.0;
	    pose.pose.position.z = track_pose.pose.position.z - 1.0;
	}
	
	if (mode_key == '1'){
	    if (get_key == 'w'){
	        get_key = '0';
	        pose.pose.position.x += 0.05;
	        print_cmd();
                print_pose(pose);
	    }
	    if (get_key == 's'){
	        get_key = '0';
	        pose.pose.position.x -= 0.05;
	        print_cmd();
                print_pose(pose);
	    }
	    if (get_key == 'a'){
	        get_key = '0';
	        pose.pose.position.y += 0.05;
	        print_cmd();
                print_pose(pose);
	    }
	    if (get_key == 'd'){
	        get_key = '0';
	        pose.pose.position.y -= 0.05;
	        print_cmd();
                print_pose(pose);
	    }
	    if (get_key == 'q'){
	        get_key = '0';
	        pose.pose.position.z += 0.05;
	        print_cmd();
                print_pose(pose);
	    }
	    if (get_key == 'e'){
	        get_key = '0';
	        pose.pose.position.z -= 0.05;
	        print_cmd();
                print_pose(pose);
	    }
	    if (get_key == 'x'){
	        get_key = '0';
	        cout << "press x Ctrl+C: exit program" << endl;
	    }
	}
	
	
	/*
	if (count < 1200) {
	    pose.pose.position.x = 0.21 + 0.75*cos(count*PI/100);
    	    pose.pose.position.y = -3.32 + 0.75*sin(count*PI/100);
	}
	else {
	    pose.pose.position.x = 0.21;
            pose.pose.position.y = -3.32;
	    pose.pose.position.z = 0.3;
	}
	*/
	
	//vel control
	/*
        cmd_vel.linear.x = 0.9*(pose.pose.position.x - path_pose.pose.position.x);
        cmd_vel.linear.y = 0.9*(pose.pose.position.y - path_pose.pose.position.y);
        cmd_vel.linear.z = 0.9*(pose.pose.position.z - path_pose.pose.position.z);
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;
        double norm_x = pow(cmd_vel.linear.x, 2);
        double norm_y = pow(cmd_vel.linear.y, 2);
        double norm_z = pow(cmd_vel.linear.z, 2);
        double norm = pow(norm_x + norm_y + norm_z, 0.5);
        if (norm > 0.5) {
            cmd_vel.linear.x = 0.5 * cmd_vel.linear.x / norm;
            cmd_vel.linear.y = 0.5 * cmd_vel.linear.y / norm;
            cmd_vel.linear.z = 0.5 * cmd_vel.linear.z / norm;
        }
        local_vel_pub.publish(cmd_vel);
	

	path_pub.publish(nav_path);*/
	
	// pos control
        local_pos_pub.publish(pose);
	
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
