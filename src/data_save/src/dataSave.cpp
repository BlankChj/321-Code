#include <ros/ros.h>
#include <kalman_filter/Vector3Stamped.h>
#include <udp_pkg/PositionVelocityAccel.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <dirent.h>
#include <regex>

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
        file_ << std::setprecision(15);
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

class DataVisionPoseRecorder : public DataRecorder
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

public:
    DataVisionPoseRecorder(const std::string &topic_name, const std::string &filename)
        : DataRecorder(filename)
    {

        if (!isOpen())
            return;

        // vison pose专用表头
        file_ << "time,posx,posy,posz,orix,oriy,oriz,oriw";
        file_ << std::endl;

        sub_ = nh_.subscribe(topic_name, 10, &DataVisionPoseRecorder::callback, this);
        ROS_INFO("DataVisionPoseRecorder: Recording %s to %s", topic_name.c_str(), fulldir_.c_str());
    }

    void callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (!isOpen())
            return;

        double time = msg->header.stamp.toSec();

        file_ << time << ","
              << msg->pose.position.x << ","
              << msg->pose.position.y << ","
              << msg->pose.position.z << ","
              << msg->pose.orientation.x << ","
              << msg->pose.orientation.y << ","
              << msg->pose.orientation.z << ","
              << msg->pose.orientation.w << std::endl;
    }
};

std::string getCurrentDateString()
{
    std::time_t t = std::time(nullptr);
    std::tm *tm = std::localtime(&t);

    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d", tm);

    return std::string(buffer);
}

std::string getMaxSuffixNumber(const std::string &dir_path)
{
    int max_suffix = -1;
    std::regex pattern(R"((.*)[_-](\d+)(?:\..*)?$)"); // 匹配 _数字 或 -数字

    DIR *dir = opendir(dir_path.c_str());
    if (dir == nullptr)
        return "000";

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        if (entry->d_name[0] == '.')
            continue;

        if (entry->d_type == DT_REG || entry->d_type == DT_UNKNOWN)
        {
            std::string filename = entry->d_name;
            std::smatch matches;

            if (std::regex_match(filename, matches, pattern))
            {
                try
                {
                    int suffix = std::stoi(matches[2].str());
                    max_suffix = std::max(max_suffix, suffix);
                }
                catch (const std::exception &e)
                {
                    // 忽略转换错误
                }
            }
        }
    }

    closedir(dir);

    std::ostringstream oss;
    oss << std::setw(3) << std::setfill('0') << max_suffix + 1;
    return oss.str();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_recorder");
    const char *homedir = getenv("HOME");
    std::string basedir = std::string(homedir) + "/DataRecord/";
    

    // 输入应为 dataSave 01
    std::string TimeDir = getCurrentDateString();
    std::string storedir = basedir+TimeDir;

    size_t last_slash = storedir.find_last_of('/');
    if (last_slash != std::string::npos)
    {
        std::string dir_path = storedir.substr(0, last_slash);

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

    std::string Num = getMaxSuffixNumber(storedir);
    // std::string Num = argv[1];
    std::string DataRawHead = "Raw_" ;
    std::string KFHead = "KF_";
    std::string RKFHead = "RKF_";
    std::string VisionPoseHead = "VisionPose_";

    // 创建不同的记录器实例
    DataRawRecorder leaderInformationRecorder("/leader/information", TimeDir + "/" + DataRawHead + Num + ".csv");
    DataFilteredRecorder kfRecorder("/leader/kf/pos", TimeDir + "/" + KFHead + Num + ".csv");
    DataFilteredRecorder rkfRecorder("/leader/rkf/pos", TimeDir + "/" + RKFHead + Num + ".csv");
    DataVisionPoseRecorder visionPoseRecorder("/mavros/vision_pose/pose", TimeDir + "/" + VisionPoseHead + Num + ".csv");

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