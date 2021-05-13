#include "ros/ros.h"
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "first_project/MotorSpeed.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define R 0.1575
#define GEAR_RATIO 40
#define APPARENT_BASELINE 0.95
#define Z 0.330364078283

double x_k;
double y_k;
double theta_k;
double x_k1;
double y_k1;
double theta_k1;
double prv_time;
double J = 0.0;
int i = 1;
tf2::Quaternion q;
geometry_msgs::PoseStamped baseline;

void callback(const first_project::MotorSpeedConstPtr& left, const first_project::MotorSpeedConstPtr& right,const geometry_msgs::PoseStampedConstPtr& pose,const ros::Publisher appa_baseline) 
{
    double v_left = (left->rpm) * 2 * M_PI * R / (60 * GEAR_RATIO);
    double v_right = (right->rpm) * 2 * M_PI * R / (60 * GEAR_RATIO);
    double omega = (v_right + v_left)/APPARENT_BASELINE;
    double v = (- v_left + v_right)/2;
    double time = pose -> header.stamp.toSec();
    double delta_time = time - prv_time;

    theta_k1 = theta_k + omega*delta_time;
    x_k1 = x_k + v*delta_time*cos(theta_k + omega*delta_time/2);
    y_k1 = y_k + v*delta_time*sin(theta_k + omega*delta_time/2);

    q.setRPY(0,0,theta_k1);

    J = J + pow(x_k1-(pose->pose.position.x),2)+pow(y_k1-(pose->pose.position.y),2)+pow(q.z()-(pose->pose.orientation.z),2);
    i++;
    if (i<500)  
        ROS_INFO("J: %f", J);

    theta_k = theta_k1;
    x_k = x_k1;
    y_k = y_k1;
    prv_time = time;    
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "baseline_estimator");

    ros::NodeHandle n;
    
    if (! n.getParam("x0", x_k))
        ROS_INFO("Error retrieving paramater x.");            
    if (! n.getParam("y0", y_k)) 
        ROS_INFO("Error retrieving paramater y.");            
    if (! n.getParam("theta0", theta_k)) 
        ROS_INFO("Error retrieving paramater theta.");
    prv_time = ros::Time::now().toSec();

    ros::Publisher appa_baseline = n.advertise<geometry_msgs::PoseStamped>("apparent_baseline", 1000);
    
    message_filters::Subscriber<first_project::MotorSpeed> sub1(n, "motor_speed_fl", 100);
    message_filters::Subscriber<first_project::MotorSpeed> sub2(n, "motor_speed_fr", 100);
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub3(n, "gt_pose", 100);
    
    typedef message_filters::sync_policies::ApproximateTime<first_project::MotorSpeed, first_project::MotorSpeed, geometry_msgs::PoseStamped> MySyncPolicy;
  
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), sub1, sub2, sub3);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, appa_baseline));

    ros::spin();

    return 0;
}