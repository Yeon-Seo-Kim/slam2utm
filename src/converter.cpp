#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath>
#include <GeographicLib/UTMUPS.hpp>

struct Cartesian{
    double x, y, z;
};

struct Quarternion{
    double qw, qx, qy, qz;
};

struct EulerAngle{
    double roll, pitch, yaw;
};

Quarternion to_quarternion(EulerAngle eangle){
    double cr = std::cos(eangle.roll*0.5);
    double sr = std::sin(eangle.roll*0.5);
    double cp = std::cos(eangle.pitch*0.5);
    double sp = std::sin(eangle.pitch*0.5);
    double cy = std::cos(eangle.yaw*0.5);
    double sy = std::sin(eangle.yaw*0.5);

    Quarternion q;
    q.qw=cr*cp*cy+sr*sp*sy;
    q.qx=sr*cp*cy-cr*sp*sy;
    q.qy=cr*sp*cy+sr*cp*sy;
    q.qz=cr*cp*sy-sr*sp*cy;

    return q;
}

EulerAngle to_euler_angle(Quarternion q){
    EulerAngle eangle;

    double sinr_cosp=2*(q.qw*q.qx+q.qy*q.qz);
    double cosr_cosp=1-2*(q.qx*q.qx+q.qy*q.qy);
    eangle.roll=std::atan2(sinr_cosp,cosr_cosp);

    double sinp = std::sqrt(1 + 2 * (q.qw * q.qy - q.qx * q.qz));
    double cosp = std::sqrt(1 - 2 * (q.qw * q.qy - q.qx * q.qz));
    eangle.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    double siny_cosp = 2 * (q.qw * q.qz + q.qx * q.qy);
    double cosy_cosp = 1 - 2 * (q.qy * q.qy + q.qz * q.qz);
    eangle.yaw = std::atan2(siny_cosp, cosy_cosp);

    return eangle;
}

class slam_converter{

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double bias_translation_x_;
    double bias_translation_y_;
    double bias_translation_z_;
    double bias_twist_roll_;
    double bias_twist_pitch_;
    double bias_twist_yaw_;


public:
    slam_converter(){
        ros::NodeHandle nh;
        sub_ = nh.subscribe("tf",10, &slam_converter::tfCallback, this);
        pub_ = nh.advertise<geometry_msgs::Twist>("current_position",10);

        nh.getParam("bias_translation_x",bias_translation_x_);
        nh.getParam("bias_translation_y",bias_translation_y_);
        nh.getParam("bias_translation_z",bias_translation_z_);
        nh.getParam("bias_twist_roll",bias_twist_roll_);
        nh.getParam("bias_twist_pitch",bias_twist_pitch_);
        nh.getParam("bias_twist_yaw",bias_twist_yaw_);
    }

    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg){
        for(const auto& transform : msg->transforms){
            geometry_msgs::TransformStamped t = transform;

            Cartesian c;

            c.x = t.transform.translation.x;
            c.y = t.transform.translation.y;
            c.z = t.transform.translation.z;

            Quarternion q;
            q.qw = t.transform.rotation.w;
            q.qx = t.transform.rotation.x;
            q.qy = t.transform.rotation.y;
            q.qz = t.transform.rotation.z;            

            EulerAngle e = to_euler_angle(q);

            double utm_x, utm_y;
            int zone;
            bool northp;
            GeographicLib::UTMUPS::Forward(c.y,c.x,zone,northp,utm_x,utm_y);

            geometry_msgs::Twist converted_utm;
            
            //converted_utm.linear; // equal to c + {bias_translation_x,y,z}
            converted_utm.linear.x=utm_x+bias_translation_x_;
            converted_utm.linear.y=utm_y+bias_translation_y_;
            converted_utm.linear.z=c.z+bias_translation_z_;
            
            //converted_utm.angular; // equal to e + {bias_twist_r,p,y}
            converted_utm.angular.x=e.roll+bias_twist_roll_;
            converted_utm.angular.y=e.pitch+bias_twist_pitch_;
            converted_utm.angular.z=e.yaw+bias_twist_yaw_;
            
            pub_.publish(converted_utm);
        }
    }

};


int main(int argc, char** argv){
    ros::init(argc,argv,"slam2utm");
    slam_converter cc;
    ros::spin();
    return 0;
}