#include "ros/ros.h"
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "first_project/MotorSpeed.h"

#define R 0.1575
#define GEAR_RATIO 38
#define APPARENT_BASELINE 1.04

void callback(const first_project::MotorSpeedConstPtr& left, const first_project::MotorSpeedConstPtr& right, const nav_msgs::OdometryConstPtr& odom) 
{
    double vx = odom->twist.twist.linear.x;
    double vy = odom->twist.twist.linear.y;
    double vel = sqrt(pow(vx,2)+pow(vy,2));
    double w = vel/R;
    
    if (((left->rpm)/(right->rpm)>0.9 || (left->rpm)/(right->rpm)<1.1) && (left->rpm)*(right->rpm)<0)  //robot is going straight
    {
        ROS_INFO("gear ratio: %f", (left->rpm)/60*2*M_PI/w);
    }

    double v_left = (left->rpm) * 2 * M_PI * R / (60 * GEAR_RATIO);
    double v_right = (right->rpm) * 2 * M_PI * R / (60 * GEAR_RATIO);
    double omega = odom->twist.twist.angular.z;
    double baseline = (v_right + v_left)/omega;

    if ((left->rpm)*(right->rpm)>0)  //robot is turning around his center
    {
        ROS_INFO("apparent baseline: %f", baseline);
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "parameters_estimation");

    ros::NodeHandle n;

    message_filters::Subscriber<first_project::MotorSpeed> sub1(n, "motor_speed_fl", 1);
    message_filters::Subscriber<first_project::MotorSpeed> sub2(n, "motor_speed_fr", 1);
    message_filters::Subscriber<nav_msgs::Odometry> sub3(n, "scout_odom", 1);
    // message_filters::Subscriber<first_project::MotorSpeed> sub4(n, "motor_speed_rr", 1);
    message_filters::TimeSynchronizer<first_project::MotorSpeed, first_project::MotorSpeed, nav_msgs::Odometry> sync(sub1, sub2, sub3, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    ros::spin();

    return 0;
    
}
