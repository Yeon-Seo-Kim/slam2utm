#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>  // Eigen 라이브러리 추가

#include <cmath>

// Euler 각도를 회전 행렬로 변환
Eigen::Matrix3d euler_to_rotation_matrix(double roll, double pitch, double yaw) {
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return R;
}

class slam_converter {
private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

public:
    Eigen::Vector3d bias_translation_;
    Eigen::Matrix3d bias_rotation_;

    slam_converter() {
        ros::NodeHandle nh;

        // Bias translation and rotation parameters
        double bias_translation_x, bias_translation_y, bias_translation_z;
        double bias_twist_roll, bias_twist_pitch, bias_twist_yaw;

        nh.getParam("/slam2utm_converter/slam2utm/bias_translation_x", bias_translation_x);
        nh.getParam("/slam2utm_converter/slam2utm/bias_translation_y", bias_translation_y);
        nh.getParam("/slam2utm_converter/slam2utm/bias_translation_z", bias_translation_z);
        nh.getParam("/slam2utm_converter/slam2utm/bias_twist_roll", bias_twist_roll);
        nh.getParam("/slam2utm_converter/slam2utm/bias_twist_pitch", bias_twist_pitch);
        nh.getParam("/slam2utm_converter/slam2utm/bias_twist_yaw", bias_twist_yaw);

        // Bias를 변환하기 위한 translation 및 rotation 행렬 생성
        bias_translation_ = Eigen::Vector3d(bias_translation_x, bias_translation_y, bias_translation_z);
        bias_rotation_ = euler_to_rotation_matrix(bias_twist_roll, bias_twist_pitch, bias_twist_yaw);

        sub_ = nh.subscribe("/lidar_odom", 10, &slam_converter::odomCallback, this);  // Odometry 메시지 구독
        pub_ = nh.advertise<geometry_msgs::Twist>("current_position", 10);  // 변환된 데이터를 publish
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Pose 정보 추출 (PoseWithCovariance에서 Pose)
        const geometry_msgs::Pose& pose = msg->pose.pose;

        // 변환 전 위치 (translation)
        Eigen::Vector3d original_position(pose.position.x, pose.position.y, pose.position.z);

        // 변환 전 orientation을 Quaternion으로 받아서 Euler 각도로 변환
        Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Eigen::Matrix3d original_rotation = quaternion.toRotationMatrix();

        // Bias rotation을 original rotation에 먼저 적용 (회전 먼저 적용)
        Eigen::Matrix3d transformed_rotation = bias_rotation_ * original_rotation;

        // Bias rotation이 적용된 좌표계로 translation도 변환
        Eigen::Vector3d transformed_position = bias_rotation_ * original_position + bias_translation_;

        // 변환된 rotation 행렬을 Euler 각도로 변환
        Eigen::Vector3d euler_angles = transformed_rotation.eulerAngles(0, 1, 2);  // Roll (X), Pitch (Y), Yaw (Z)

        // 변환된 값을 Twist 메시지로 변환하여 송출
        geometry_msgs::Twist converted_twist;
        
        // 변환된 translation 적용
        converted_twist.linear.x = transformed_position.x();
        converted_twist.linear.y = transformed_position.y();
        converted_twist.linear.z = transformed_position.z();

        // 변환된 Euler 각도 (Roll, Pitch, Yaw)를 Twist 메시지에 적용
        converted_twist.angular.x = euler_angles[0];  // Roll
        converted_twist.angular.y = euler_angles[1];  // Pitch
        converted_twist.angular.z = euler_angles[2];  // Yaw

        pub_.publish(converted_twist);  // 송출
        }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "slam2utm");
    slam_converter converter;
    ros::spin();
    return 0;
}
