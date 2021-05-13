#include "ros/ros.h"
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TwistStamped.h>
#include "first_project/MotorSpeed.h"

#define R 0.1575
#define GEAR_RATIO 38
#define APPARENT_BASELINE 1.04

void callback(const first_project::MotorSpeedConstPtr& left, const first_project::MotorSpeedConstPtr& right, const ros::Publisher scout_velocity) 
{
    double v_left = (left->rpm) * 2 * M_PI * R / (60 * GEAR_RATIO);
    double v_right = (right->rpm) * 2 * M_PI * R / (60 * GEAR_RATIO);
    double omega = (v_right + v_left)/APPARENT_BASELINE;
    double v = (- v_left + v_right)/2;                   

    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "scout_frame";                   

    msg.twist.linear.x = v;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = omega;

    scout_velocity.publish(msg);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "syncronizer");

    ros::NodeHandle n;

    ros::Publisher scout_velocity = n.advertise<geometry_msgs::TwistStamped>("scout_velocity", 1000);

    message_filters::Subscriber<first_project::MotorSpeed> sub1(n, "motor_speed_fl", 1);
    message_filters::Subscriber<first_project::MotorSpeed> sub2(n, "motor_speed_fr", 1);
    // message_filters::Subscriber<first_project::MotorSpeed> sub3(n, "motr_speed_rl", 1);
    // message_filters::Subscriber<first_project::MotorSpeed> sub4(n, "motor_speed_rr", 1);
    message_filters::TimeSynchronizer<first_project::MotorSpeed, first_project::MotorSpeed> sync(sub1, sub2, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, scout_velocity));

    ros::spin();

    return 0;
    
}
