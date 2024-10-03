#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>

#include <cmath>


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
    ros::Publisher path_pub_;
    nav_msgs::Path path_;


public:

    double bias_translation_x_;
    double bias_translation_y_;
    double bias_translation_z_;
    double bias_twist_roll_;
    double bias_twist_pitch_;
    double bias_twist_yaw_;


    slam_converter(){
         ros::NodeHandle nh;
        nh.getParam("/slam2utm_converter/slam2utm/bias_translation_x",bias_translation_x_);
        nh.getParam("/slam2utm_converter/slam2utm/bias_translation_y",bias_translation_y_);
        nh.getParam("/slam2utm_converter/slam2utm/bias_translation_z",bias_translation_z_);
        nh.getParam("/slam2utm_converter/slam2utm/bias_twist_roll",bias_twist_roll_);
        nh.getParam("/slam2utm_converter/slam2utm/bias_twist_pitch",bias_twist_pitch_);
        nh.getParam("/slam2utm_converter/slam2utm/bias_twist_yaw",bias_twist_yaw_);
        sub_ = nh.subscribe("tf",8, &slam_converter::tfCallback, this);
        pub_ = nh.advertise<geometry_msgs::Twist>("current_position",10);
        ros::Rate rate(8);
        std::cout<<bias_twist_yaw_<<std::endl;
        

        ROS_INFO("Bias translation x: %f", bias_translation_x_);
        ROS_INFO("Bias translation y: %f", bias_translation_y_);
        ROS_INFO("Bias translation z: %f", bias_translation_z_);
        ROS_INFO("Bias twist roll: %f", bias_twist_roll_);
        ROS_INFO("Bias twist pitch: %f", bias_twist_pitch_);
        ROS_INFO("Bias twist yaw: %f", bias_twist_yaw_);



    }

    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg){
        // ROS_INFO("Bias translation x: %f", bias_translation_x_);
        // ROS_INFO("Bias translation y: %f", bias_translation_y_);
        // ROS_INFO("Bias translation z: %f", bias_translation_z_);
        // ROS_INFO("Bias twist roll: %f", bias_twist_roll_);
        // ROS_INFO("Bias twist pitch: %f", bias_twist_pitch_);
        // ROS_INFO("Bias twist yaw: %f", bias_twist_yaw_);
        for(std::vector<geometry_msgs::TransformStamped>::const_iterator it = msg->transforms.begin(); it != msg->transforms.end(); ++it){
            geometry_msgs::TransformStamped t = *it;

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

            geometry_msgs::Twist converted_utm;
            
            //converted_utm.linear; // equal to c + {bias_translation_x,y,z}
            converted_utm.linear.x = c.x + bias_translation_x_;
            converted_utm.linear.y = c.y + bias_translation_y_;
            converted_utm.linear.z = c.z + bias_translation_z_;
            
            //converted_utm.angular; // equal to e + {bias_twist_r,p,y}
            converted_utm.angular.x = e.roll + bias_twist_roll_;
            converted_utm.angular.y = e.pitch + bias_twist_pitch_;
            converted_utm.angular.z = e.yaw + bias_twist_yaw_;
            

            /**
             * 
            //ROS_INFO("ORIGINAL LINEAR X: %f", c.x);
            //ROS_INFO("ORIGINAL LINEAR Y: %f", c.y);
            //ROS_INFO("ORIGINAL LINEAR Z: %f", c.z);
            //ROS_INFO("ORIGINAL ANGULAR R: %f", e.roll);
            //ROS_INFO("ORIGINAL ANGULAR P: %f", e.pitch);
            //ROS_INFO("ORIGINAL ANGULAR y: %f", e.yaw);

            double original[6] = {c.x,c.y,c.z,e.roll,e.pitch,e.yaw};
            double converted[6] =  {converted_utm.linear.x,converted_utm.linear.y,converted_utm.linear.z,converted_utm.angular.x, converted_utm.angular.y, converted_utm.angular.z};

            for(auto origin : original){
                ROS_INFO("ORIGINAL STATUS: %f", origin);
            }

            ROS_INFO("ORIGINAL LINEAR X: %f\nORIGINAL LINEAR Y: %f\nORIGINAL LINEAR Z: %f\nORIGINAL ANGULAR R: %f\nORIGINAL ANGULAR P: %f\nORIGINAL ANGULAR y: %f\n",c.x,c.y,c.z,e.roll,e.pitch,e.yaw);
            ROS_INFO("CONVERTED LINEAR X: %f\nCONVERTED LINEAR Y: %f\nCONVERTED LINEAR Z: %f\nCONVERTED ANGULAR R: %f\nCONVERTED ANGULAR P: %f\nCONVERTED ANGULAR y: %f\n",converted_utm.linear.x,converted_utm.linear.y,converted_utm.linear.z,converted_utm.angular.x, converted_utm.angular.y, converted_utm.angular.z);

            ROS_INFO("\n");
            for(auto convert : converted){
                ROS_INFO("CONVERTED STATUS: %f", convert);
            }
            

            //ROS_INFO("CONVERTED LINEAR X: %f", converted_utm.linear.x);
            //ROS_INFO("CONVERTED LINEAR Y: %f", converted_utm.linear.y);
            //ROS_INFO("CONVERTED LINEAR Z: %f", converted_utm.linear.z);
            //ROS_INFO("CONVERTED ANGULAR R: %f", converted_utm.angular.x);
            //ROS_INFO("CONVERTED ANGULAR P: %f", converted_utm.angular.y);
            //ROS_INFO("CONVERTED ANGULAR y: %f", converted_utm.angular.z);

            */

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6);
            oss << "\n";
            oss << "ORIGINAL LINEAR X: " << c.x << "\n";
            oss << "ORIGINAL LINEAR Y: " << c.y << "\n";
            oss << "ORIGINAL LINEAR Z: " << c.z << "\n";
            oss << "ORIGINAL ANGULAR R: " << e.roll << "\n";
            oss << "ORIGINAL ANGULAR P: " << e.pitch << "\n";
            oss << "ORIGINAL ANGULAR Y: " << e.yaw << "\n";
            oss << "CONVERTED LINEAR X: " << converted_utm.linear.x << "\n";
            oss << "CONVERTED LINEAR Y: " << converted_utm.linear.y << "\n";
            oss << "CONVERTED LINEAR Z: " << converted_utm.linear.z << "\n";
            oss << "CONVERTED ANGULAR R: " << converted_utm.angular.x << "\n";
            oss << "CONVERTED ANGULAR P: " << converted_utm.angular.y << "\n";
            oss << "CONVERTED ANGULAR Y: " << converted_utm.angular.z << "\n";

            ROS_INFO("%s", oss.str().c_str());
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