#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_messages/Num.h"

void chatterCallback(const custom_messages::Num::ConstPtr& msg){
  ROS_INFO("I heard: [%d]", msg->num);
}

int main(int argc, char **argv){
  	
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  	ros::spin();

  return 0;
}


