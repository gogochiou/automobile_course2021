#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <math.h>
#include <geometry_msgs/Twist.h>
using namespace std;

#define PI 3.14159265

ros::Publisher twist_pub;

struct position{
    double x;
    double y;
    double theta;
};
struct position cur_pos;
struct position goal_pos;

double theta_domain_conversion(double input_theta){
    double in ;
    double output_theta;
    in = input_theta;
    if (in >=0 ){

        while(in >= 2*PI){
            in -= 2*PI;
        }

        if(in > PI){
            in -= 2*PI;
            output_theta = in;
        }

        else{
            output_theta = in;
        }

    }
    else{
        in = in * -1;

        while(in >= 2*PI){
            in -= 2*PI;
        }

        if(in > PI){
            in -= 2*PI;
            in *= -1;
            output_theta = in;
        }

        else{
            in *= -1;
            output_theta = in;
        }

    }
    return output_theta;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    // ros::Subscriber pose_sub = nh.subscribe("/robot_pose", 10, poseCallback。oalCallback);
    // ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); 
    ros::spin();
    return 0;
}