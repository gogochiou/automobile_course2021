#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <queue>
#include <math.h>
#include <geometry_msgs/Twist.h>
using namespace std;

#define PI 3.14159265
#define xy_margin 0.1
#define theta_margin 0.1


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

double getDistance(double x1, double y1, double x2, double y2){
    double dis = 0;
    dis = sqrt(pow((x1-x2), 2) + pow((y1-y2), 2) );
    return dis;
}

void goalCallback(const geometry_msgs::PoseStamped &goal){
    tf::Quaternion q1(goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
    tf::Matrix3x3 Matrix;
    Matrix.setRotation(q1);
    tfScalar m_yaw,m_pitch,m_roll;
    Matrix.getEulerYPR(m_yaw,m_pitch,m_roll);
    goal_pos.x = goal.pose.position.x;
    goal_pos.y = goal.pose.position.y;
    goal_pos.theta = m_yaw;

    ROS_INFO("Target Achieved = [%f %f %f]", goal_pos.x, goal_pos.y, goal_pos.theta);
}

bool xy_goalreached(position cur_pos, position goal_pos){
    if( getDistance(goal_pos.x, goal_pos.y, cur_pos.x, cur_pos.y) < xy_margin){
        return true;
    }
    else
        return false;
}

bool theta_goalreached(position cur_pos, position goal_pos){
    if( abs(goal_pos.theta - cur_pos.theta) < theta_margin ){
        return true;
    }
    else
        return false;
}

void commandDrive(ros::Publisher twist_pub, double linear, double angular){
   geometry_msgs::Twist twist;
   twist.linear.x = linear;
   twist.linear.y = 0;
   twist.linear.z = 0;
   twist.angular.x = 0;
   twist.angular.y = 0;
   twist.angular.z = angular;
   twist_pub.publish(twist);
}

void poseCallback(const geometry_msgs::Twist &pose){
    // cout << "%f",pose->x << endl;
    cur_pos.x = pose.linear.x;
    cur_pos.y = pose.linear.y;
    cur_pos.theta = theta_domain_conversion(pose.angular.z);

    double k_alpha = .8;
    double k_beta = -0.15;
    double k_rho = .3;
    double linear_v = 0;
    double angular_v = 0;
    double err_dis = 0;
    double err_alpha = 0;
    double err_beta = 0;

    ROS_INFO("theta_goalreached %d", theta_goalreached(cur_pos, goal_pos));
    ROS_INFO("xy_goalreached %d", xy_goalreached(cur_pos, goal_pos));

    if (!(xy_goalreached(cur_pos, goal_pos))){
        //linear
        err_dis = getDistance(cur_pos.x, cur_pos.y, goal_pos.x, goal_pos.y);
        // ROS_INFO("err_dis = %f", err_dis);
        linear_v = k_rho * err_dis;
    }
    else{
        ROS_INFO("aaaaaaaaaaaaa");
        k_beta *=2;
        k_alpha = 0;
        linear_v = 0;
    }
    if(!theta_goalreached(cur_pos, goal_pos) || (!(xy_goalreached(cur_pos, goal_pos)) && theta_goalreached(cur_pos, goal_pos))){
        err_beta = cur_pos.theta - goal_pos.theta;
        err_alpha = atan2((goal_pos.y - cur_pos.y), (goal_pos.x - cur_pos.x)) - cur_pos.theta;
        // ROS_INFO("err_alpha = %f", err_alpha);
        ROS_INFO("err_beta = %f", err_beta);
        // angular_v = theta_domain_conversion(k_alpha * err_alpha + k_beta * err_beta);
        angular_v = k_alpha * theta_domain_conversion(err_alpha) + k_beta * theta_domain_conversion(err_beta);        
    }
    else if( xy_goalreached(cur_pos, goal_pos) && theta_goalreached(cur_pos, goal_pos) ){
        angular_v = 0;
        k_beta = k_beta/2;
        ROS_INFO("GOAL REACHED!");   
    }
    commandDrive(twist_pub, linear_v, angular_v);
    ROS_INFO("k_beta = %f", k_beta);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/robot_pose", 10, poseCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goalCallback);
    // ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); 
    ros::spin();
    return 0;
}