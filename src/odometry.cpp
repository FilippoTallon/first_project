#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <first_project/methodsConfig.h>

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
      
    f = boost::bind(&Odometry::setMethod, this,_1, _2);
    server.setCallback(f);

      sub = n.subscribe("/scout_velocity", 1000, &Odometry::callback, this);

      odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    }
    
    void setMethod(first_project::methodsConfig &config, uint32_t level) 
    {
      if(config.method == 0)
        eulerMethod = true;
      else eulerMethod = false;
      ROS_INFO("Reconfigure Request: %d", config.method);
      ROS_INFO(eulerMethod?"Euler":"Runge-Kutta");
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

        if(eulerMethod)
          euler(msg, v, omega, delta_time);
        else kutta(msg, v, omega, delta_time);

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
    dynamic_reconfigure::Server<first_project::methodsConfig> server;
    dynamic_reconfigure::Server<first_project::methodsConfig>::CallbackType f;
    double x_k;
    double y_k;
    double theta_k;
    double x_k1;
    double y_k1;
    double theta_k1;
    double prv_time = ros::Time::now().toSec();
    bool eulerMethod = true;
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "odometry");
    
    Odometry odo();

    ros::spin();

    return 0;
}