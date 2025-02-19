#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

ros::Publisher pub_global_smooth_odometry;
ros::Publisher pub_global_vio_path;
nav_msgs::Path path;

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

Eigen::Vector3d imu_acceleration = Eigen::Vector3d::Zero();
Eigen::Vector3d imu_angular_velocity = Eigen::Vector3d::Zero();

// 状态量
Vector6d X;
Matrix6d P;
Matrix6d F;
Eigen::Matrix<double, 3, 6> H;
Matrix6d Q;
Eigen::Matrix3d R;

// 平滑调整参数
double alpha = 0.7;  // 平滑因子 (0.7~0.9)

// 获取速度和加速度
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu_acceleration << msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z;
    
    imu_angular_velocity << msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z;
}

// **动态调整噪声 R **
void updateMeasurementNoise(const Eigen::Vector3d &Z) {
    static Eigen::Vector3d last_Z = Z;
    Eigen::Vector3d diff = Z - last_Z;
    double change_rate = diff.norm();

    // **动态调整 R：变化剧烈时增大 R**
    double r_factor = std::min(5.0, 1.0 + change_rate * 2.0);
    R = Eigen::Matrix3d::Identity() * (0.1 * r_factor);

    last_Z = Z;
}

// **初始化卡尔曼滤波**
void initKalmanFilter() {
    X.setZero();
    P = Matrix6d::Identity() * 1.0;

    F << 1, 0, 0, 0.1, 0, 0,
         0, 1, 0, 0, 0.1, 0,
         0, 0, 1, 0, 0, 0.1,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0;

    Q = Matrix6d::Identity() * 0.001;
    R = Eigen::Matrix3d::Identity() * 0.1;
}

// **计算 Mahalanobis 距离**
double mahalanobisDistance(const Eigen::Vector3d &Z, const Eigen::Vector3d &Z_pred) {
    Eigen::Vector3d diff = Z - Z_pred;
    Eigen::Matrix3d S = H * P * H.transpose() + R;
    return sqrt(diff.transpose() * S.inverse() * diff);
}

// **处理 Odom 回调**
void global_vio_Callback(const nav_msgs::Odometry::ConstPtr &msg) {
    double dt = 0.1;
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;

    // **预测**
    X = F * X;
    P = F * P * F.transpose() + Q;

    // **观测值**
    Eigen::Vector3d Z;
    Z << msg->pose.pose.position.x,
         msg->pose.pose.position.y,
         msg->pose.pose.position.z;

    // **计算预测观测值**
    Eigen::Vector3d Z_pred = H * X;


    // 计算机动状态
    double acc_norm = imu_acceleration.norm();  // 线加速度
    double gyro_norm = imu_angular_velocity.norm();  // 角速度

    // **异常值检测**
    double maha_dist = mahalanobisDistance(Z, Z_pred);
    if (maha_dist > 3.0) {  // 如果偏离过大，减小修正幅度
        R *= 5;
    } else {
        updateMeasurementNoise(Z);
    }

    // **计算卡尔曼增益**
    Eigen::Matrix3d S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();

    // **平滑卡尔曼增益**
    static Eigen::Matrix<double, 6, 3> last_K = K;
    K = alpha * last_K + (1 - alpha) * K;
    last_K = K;

    // **更新状态**
    X = X + K * (Z - Z_pred);
    P = (Matrix6d::Identity() - K * H) * P;

    // **发布平滑 Odom**
    nav_msgs::Odometry filtered_odom;
    filtered_odom.header = msg->header;
    filtered_odom.pose.pose.position.x = X(0);
    filtered_odom.pose.pose.position.y = X(1);
    filtered_odom.pose.pose.position.z = X(2);
    filtered_odom.twist.twist.linear.x = X(3);
    filtered_odom.twist.twist.linear.y = X(4);
    filtered_odom.twist.twist.linear.z = X(5);
    pub_global_smooth_odometry.publish(filtered_odom);

    // **发布轨迹**
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position.x = X(0);
    pose.pose.position.y = X(1);
    pose.pose.position.z = X(2);
    path.header = msg->header;
    path.poses.push_back(pose);
    pub_global_vio_path.publish(path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_smooth_fusion");
    ros::NodeHandle nh;

    initKalmanFilter();

    // 订阅全局里程计
    ros::Subscriber sub_global_vio = nh.subscribe("/global_fusion/global_odometry", 100, global_vio_Callback);
    // 订阅IMU数据
    ros::Subscriber imu_sub = nh.subscribe("/imu/filtered", 100, imuCallback);
    // 发布滤波全局里程计
    pub_global_smooth_odometry = nh.advertise<nav_msgs::Odometry>("/global_smooth_odometry", 10);
    // 发布滤波全局轨迹
    pub_global_vio_path = nh.advertise<nav_msgs::Path>("/global_smooth_path", 10);

    ros::spin();
    return 0;
}