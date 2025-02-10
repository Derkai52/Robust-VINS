#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <sstream>
#include <vector>

struct GroundTruthData {
    double timestamp;
    double x, y, z;
    double qx, qy, qz, qw;
};

class GroundTruthPublisher {
public:
    GroundTruthPublisher(ros::NodeHandle& nh) : nh_(nh) {
        // 读取参数中的真值数据文件路径
        nh_.param<std::string>("ground_truth_file", gt_file_path_, "/home/tk/robust_vins/src/robust_vins/config/M300/100_4.txt");

        // 读取真值轨迹数据
        if (!loadGroundTruthData(gt_file_path_)) {
            ROS_ERROR("Failed to load ground truth data from %s", gt_file_path_.c_str());
            ros::shutdown();
        }

        // 订阅图像话题，提取时间戳
        image_sub_ = nh_.subscribe("/left/downsample_raw", 10, &GroundTruthPublisher::imageCallback, this);

        // 发布 PoseStamped 和 Path 轨迹
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/ground_truth_pose", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/ground_truth_path", 10);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher pose_pub_, path_pub_;
    std::vector<GroundTruthData> ground_truth_;
    nav_msgs::Path path_;
    std::string gt_file_path_;

    bool loadGroundTruthData(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            ROS_ERROR("Cannot open ground truth file: %s", file_path.c_str());
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            GroundTruthData data;
            ss >> data.timestamp >> data.x >> data.y >> data.z >> data.qx >> data.qy >> data.qz >> data.qw;
            ground_truth_.push_back(data);
        }
        file.close();
        ROS_INFO("Loaded %zu ground truth entries.", ground_truth_.size());
        return true;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        double query_time = msg->header.stamp.toSec();
        GroundTruthData closest_data;
        double min_time_diff = 1e6; // 初始化一个较大值

        // 查找最邻近时间戳的真值数据
        for (const auto& data : ground_truth_) {
            double time_diff = std::abs(data.timestamp - query_time);
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_data = data;
            }
        }

        // 如果误差 > 10ms，则跳过发布
        if (min_time_diff > 0.01) {
            // ROS_WARN("No close timestamp found within 3ms, skipping...");
            return;
        }

        // 发布 PoseStamped
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time().fromSec(closest_data.timestamp);
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = closest_data.x;
        pose_msg.pose.position.y = closest_data.y;
        pose_msg.pose.position.z = closest_data.z;
        pose_msg.pose.orientation.x = closest_data.qx;
        pose_msg.pose.orientation.y = closest_data.qy;
        pose_msg.pose.orientation.z = closest_data.qz;
        pose_msg.pose.orientation.w = closest_data.qw;
        pose_pub_.publish(pose_msg);

        // 更新并发布 Path
        path_.header = pose_msg.header;
        path_.poses.push_back(pose_msg);
        path_pub_.publish(path_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_truth_publisher");
    ros::NodeHandle nh;
    GroundTruthPublisher publisher(nh);
    ros::spin();
    return 0;
}