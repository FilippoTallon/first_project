#include "ros/ros.h"
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "first_project/MotorSpeed.h"
#include <message_filters/sync_policies/approximate_time.h>

#define R 0.1575
#define GEAR_RATIO 38
#define T 0.00036

double theta_k;
double time_k;
std_msgs::Float64 baseline;

void callback(const first_project::MotorSpeedConstPtr& left, 
              const first_project::MotorSpeedConstPtr& right,
              const geometry_msgs::PoseStampedConstPtr& pose,
              const ros::Publisher appa_baseline) {


    double v_left = (left->rpm) * 2 * M_PI * R / (60 * GEAR_RATIO);
    double v_right = (right->rpm) * 2 * M_PI * R / (60 * GEAR_RATIO);

    double theta_k1 = pose -> pose.orientation.z;
    double time_k1 = pose -> header.stamp.toSec();
    
    double delta_time = time_k1 - time_k;

    double omega_k = (theta_k1 - theta_k) / delta_time;

    baseline.data = (v_right + v_left) / omega_k;

    // if (v_left/v_right < 1.2 && v_left/v_right > 0.8 && v_left * v_right > 0){
        
    // }
    //ROS_INFO("APPARENT BASELINE: %f", baseline.data);
    // ROS_INFO("theta_k+1: %f, theta_k: %f", theta_k1, theta_k);
    // ROS_INFO("time_k+1: %f, time_k: %f", time_k1, time_k);
    // ROS_INFO("delta_t: %f", time_k);
    // ROS_INFO("omega: %f", omega_k);
    
    theta_k = theta_k1;
    time_k = time_k1;
    appa_baseline.publish(baseline);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "baseline_calculator");

    ros::NodeHandle n;
    
    if (! n.getParam("theta0", theta_k)) 
        ROS_INFO("Error retrieving paramater theta.");
    time_k = ros::Time::now().toSec();

    ros::Publisher appa_baseline = n.advertise<std_msgs::Float64>("apparent_baseline", 1000);

    
    message_filters::Subscriber<first_project::MotorSpeed> sub1(n, "motor_speed_fl", 100);
    message_filters::Subscriber<first_project::MotorSpeed> sub2(n, "motor_speed_fr", 100);
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub3(n, "gt_pose", 100);
    
    typedef message_filters::sync_policies
      ::ApproximateTime<first_project::MotorSpeed, first_project::MotorSpeed, geometry_msgs::PoseStamped> MySyncPolicy;
  
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), sub1, sub2, sub3);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, appa_baseline));


    ros::spin();

    return 0;
}