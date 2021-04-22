#include "ros/ros.h"
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TwistStamped.h>
#include "first_project/MotorSpeed.h"

#define R 0.1575
#define GEAR_RATIO 38
#define APPA 1.04

void callback(const robotics_first::MotorSpeedConstPtr& left, 
              const robotics_first::MotorSpeedConstPtr& right,
              const ros::Publisher twist) {

    float v_left = (left->rpm) * 2 * M_PI * R / (60 * GEAR_RATIO);
    float v_right = (right->rpm) * 2 * M_PI * R / (60 * GEAR_RATIO);
    float w = (v_right + v_left)/APPA;
    float v_x = (- v_left + v_right)/2;

    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "fcu";
    msg.twist.linear.x = v_x;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = w;

    twist.publish(msg);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "syncronizer");

    ros::NodeHandle n;

    ros::Publisher twist = n.advertise<geometry_msgs::TwistStamped>("twist", 1000);

    message_filters::Subscriber<robotics_first::MotorSpeed> sub1(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_first::MotorSpeed> sub2(n, "motor_speed_fr", 1);
    // message_filters::Subscriber<robotics_first::MotorSpeed> sub3(n, "motr_speed_rl", 1);
    // message_filters::Subscriber<robotics_first::MotorSpeed> sub4(n, "motor_speed_rr", 1);
    message_filters::TimeSynchronizer<robotics_first::MotorSpeed, 
                                        robotics_first::MotorSpeed> sync(sub1, sub2, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2, twist));

    ros::spin();

    return 0;
    
}
