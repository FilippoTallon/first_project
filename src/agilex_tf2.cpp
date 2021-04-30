#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

class tf_sub_pub
{
public:
  tf_sub_pub() {
    sub = n.subscribe("odometry", 1000, &tf_sub_pub::callback, this);
  }

  void callback(const nav_msgs::Odometry::ConstPtr& msg){
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "agilex";
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    /*tf2::Quaternion q;
    q.setRPY(0, 0, msg->pose.pose.orientation.z);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();*/
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
    br.sendTransform(transformStamped);
  }

private:
  ros::NodeHandle n; 
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  ros::Subscriber sub;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_and_publish");
  tf_sub_pub my_tf_sub_bub;
  ros::spin();
  return 0;
}
