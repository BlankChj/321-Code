#include "KF.h"
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "kalman_filter");
    ros::NodeHandle nh;

    KF kf(nh);
    ros::Rate lp(20);

    while (ros::ok()){
        ros::spinOnce();
        kf.run();

        lp.sleep();
        
    }
}