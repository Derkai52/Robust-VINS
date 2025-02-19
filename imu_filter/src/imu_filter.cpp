#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

class IMUKalmanFilter
{
public:
    IMUKalmanFilter()
    {
        // 初始状态
        x_ = Eigen::VectorXd::Zero(6); // [ax, ay, az, wx, wy, wz]
        P_ = Eigen::MatrixXd::Identity(6, 6);
        Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01; // 过程噪声
        R_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;  // 观测噪声
    }

    Eigen::VectorXd filter(const Eigen::VectorXd &z)
    {
        // 预测步骤
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd F = I; // 状态转移矩阵（假设匀速模型）
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q_;
        
        // 更新步骤
        Eigen::MatrixXd H = I; // 观测矩阵（直接观测加速度和角速度）
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        x_ = x_ + K * (z - H * x_);
        P_ = (I - K * H) * P_;

        return x_;
    }

private:
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_, Q_, R_;
};

class IMUNode
{
public:
    IMUNode()
    {
        ros::NodeHandle nh;
        imu_sub_ = nh.subscribe("/imu/imu/data", 10, &IMUNode::imuCallback, this);
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/filtered", 10);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        Eigen::VectorXd z(6);
        z << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        
        Eigen::VectorXd filtered = kf_.filter(z);
        
        sensor_msgs::Imu filtered_msg = *msg;
        filtered_msg.linear_acceleration.x = filtered(0);
        filtered_msg.linear_acceleration.y = filtered(1);
        filtered_msg.linear_acceleration.z = filtered(2);
        filtered_msg.angular_velocity.x = filtered(3);
        filtered_msg.angular_velocity.y = filtered(4);
        filtered_msg.angular_velocity.z = filtered(5);
        
        imu_pub_.publish(filtered_msg);
    }

private:
    ros::Subscriber imu_sub_;
    ros::Publisher imu_pub_;
    IMUKalmanFilter kf_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_filter");
    IMUNode node;
    ros::spin();
    return 0;
}
