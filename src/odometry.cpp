#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define T_s 0.1

class Odometry
{
    public:
    Odometry()
    {
        if (! n.getParam("x0", x_k))
          ROS_INFO("Error retrieving paramater x.");
            
        if (! n.getParam("y0", y_k)) 
          ROS_INFO("Error retrieving paramater y.");
            
        if (! n.getParam("theta0", y_k)) 
          ROS_INFO("Error retrieving paramater theta.");
            
        sub = n.subscribe("/scout_velocity", 1000, &Odometry::callback, this);

        odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    }

    
    void euler(const geometry_msgs::TwistStampedConstPtr& msg, double v, double omega, double time)
    {
        theta_k1 = theta_k + omega*time;
        x_k1 = x_k + v*time*cos(theta_k);
        y_k1 = y_k + v*time*sin(theta_k);

    }

    void kutta(const geometry_msgs::TwistStampedConstPtr& msg, double v, double omega, double time)
    {
        theta_k1 = theta_k + omega*time;
        x_k1 = x_k + v*time*cos(theta_k + omega*time/2);
        y_k1 = y_k + v*time*sin(theta_k + omega*time/2);
    }

    void callback(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        double v = msg -> twist.linear.x;
        double omega = msg -> twist.angular.z;
        double time = msg -> header.stamp.toSec();
        double delta_time = time - prv_time;

        euler(msg, v, omega, delta_time);
        //kutta(msg, v, omega, delta_time);

        odo_msg.child_frame_id = "scout_frame";
        odo_msg.header.frame_id = "world";
        odo_msg.header.stamp = ros::Time::now();
        odo_msg.pose.pose.position.x = x_k1;
        odo_msg.pose.pose.position.y = y_k1;

        q.setRPY(0,0,theta_k1);
        odo_msg.pose.pose.orientation.x = q.x();
        odo_msg.pose.pose.orientation.y = q.y();
        odo_msg.pose.pose.orientation.z = q.z();
        odo_msg.pose.pose.orientation.w = q.w();

        odometry.publish(odo_msg);

        theta_k = theta_k1;
        x_k = x_k1;
        y_k = y_k1;
        prv_time = time;
    }

    private:
    ros::NodeHandle n;
    ros::Publisher odometry;
    ros::Subscriber sub;
    nav_msgs::Odometry odo_msg; 
    tf2::Quaternion q;
    double x_k;
    double y_k;
    double theta_k;
    double x_k1;
    double y_k1;
    double theta_k1;
    double prv_time = ros::Time::now().toSec();
};


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "odometry");
    
    Odometry odo;

    ros::spin();

    return 0;
}