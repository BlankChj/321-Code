#include <ros/ros.h>
#include <kalman_filter/Vector3Stamped.h>
#include <udp_pkg/PositionVelocityAccel.h>
#include <fstream>
#include <iostream>

class DataRecorder
{
protected:
    std::ofstream file_;
    std::string filename_;
    std::string basedir_;
    std::string fulldir_;
public:
    DataRecorder(const std::string &filename) : filename_(filename)
    {
        const char *home_dir = getenv("HOME");
        if (home_dir)
        {
            basedir_ = std::string(home_dir) + "/DataRecord/";
        }
        else
        {
            basedir_ = "./DataRecord/"; // 备用方案
            ROS_WARN("HOME environment variable not set, using current directory");
        }

        fulldir_ = basedir_ + filename_.c_str();

        size_t last_slash = fulldir_.find_last_of('/');
        if (last_slash != std::string::npos)
        {
            std::string dir_path = fulldir_.substr(0, last_slash);

            // 递归创建目录
            std::string cmd = "mkdir -p \"" + dir_path + "\"";
            int result = system(cmd.c_str());
            if (result != 0)
            {
                ROS_WARN("Failed to create directory: %s", dir_path.c_str());
            }
            else
            {
                ROS_INFO("Created directory: %s", dir_path.c_str());
            }
        }


        file_.open(fulldir_);
        if (!file_.is_open())
        {
            ROS_ERROR("Failed to open file: %s", fulldir_.c_str());
        } else {
            ROS_INFO("Open file successfully!");
        }
    }

    virtual ~DataRecorder()
    {
        if (file_.is_open())
        {
            file_.close();
        }
    }

    bool isOpen() const { return file_.is_open(); }
};

class DataRawRecorder : public DataRecorder
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

public:
    DataRawRecorder(const std::string &topic_name, const std::string &filename)
        : DataRecorder(filename)
    {

        if (!isOpen())
            return;

        // 传递原数据专用表头
        file_ << "frame_id,stamp,x_pos,y_pos,z_pos,x_ori,y_ori,z_ori,w_ori,x_vel,y_vel,z_vel,x_acc,y_acc,z_acc";
        file_ << std::endl;

        sub_ = nh_.subscribe(topic_name, 10, &DataRawRecorder::callback, this);
        ROS_INFO("DataRawRecorder: Recording %s to %s", topic_name.c_str(), fulldir_.c_str());
    }

    void callback(const udp_pkg::PositionVelocityAccel::ConstPtr &msg)
    {
        if (!isOpen())
            return;

        file_ << msg->frame_id << ","
              << msg->stamp << ","
              << msg->x_pos << ","
              << msg->y_pos << ","
              << msg->z_pos << ","
              << msg->x_ori << ","
              << msg->y_ori << ","
              << msg->z_ori << ","
              << msg->w_ori << ","
              << msg->x_vel << ","
              << msg->y_vel << ","
              << msg->z_vel << ","
              << msg->x_acc << ","
              << msg->y_acc << ","
              << msg->z_acc;

        file_ << std::endl;
    }
};

class DataFilteredRecorder : public DataRecorder
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

public:
    DataFilteredRecorder(const std::string &topic_name, const std::string &filename)
        : DataRecorder(filename)
    {

        if (!isOpen())
            return;

        // 滤波器专用表头
        file_ << "time,x,y,z";
        file_ << std::endl;

        sub_ = nh_.subscribe(topic_name, 10, &DataFilteredRecorder::callback, this);
        ROS_INFO("DataFilteredRecorder: Recording %s to %s", topic_name.c_str(), fulldir_.c_str());
    }

    void callback(const kalman_filter::Vector3Stamped::ConstPtr &msg)
    {
        if (!isOpen())
            return;

        file_ << msg->time << ","
              << msg->x << ","
              << msg->y << ","
              << msg->z << std::endl;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_recorder");

    // 输入应为 dataSave 20251223 01
    std::string TimeDir = argv[1];
    std::string Num = argv[2];
    std::string DataRawHead = "Raw_.csv" ;
    std::string KFHead = "KF_.csv";
    std::string RKFHead = "RKF_.csv";
    // 创建不同的记录器实例
    DataRawRecorder leaderInformationRecorder("/leader/information", TimeDir + "/" + DataRawHead + Num);
    DataFilteredRecorder kfRecorder("/leader/kf/pos", TimeDir + "/" + KFHead + Num);
    DataFilteredRecorder rkfRecorder("/leader/kf/vel", TimeDir + "/" + RKFHead + Num);

    // 或者根据参数决定创建哪种记录器
    // if (argc > 1)
    // {
    //     std::string topic_type = argv[1];
    //     if (topic_type == "laser")
    //     {
    //         LaserRecorder recorder("/scan", "laser.csv");
    //         ros::spin();
    //     }
    //     else if (topic_type == "pose")
    //     {
    //         PoseRecorder recorder("/pose", "pose.csv");
    //         ros::spin();
    //     }
    // }
    // else
    // {
    //     // 同时运行多个记录器
    //     ros::spin();
    // }

    ros::spin();
    
    return 0;
}