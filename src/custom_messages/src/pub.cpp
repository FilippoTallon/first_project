#include "ros/ros.h"
#include "custom_messages/Num.h"

#include <sstream>

int main(int argc, char **argv){
    
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<custom_messages::Num>("chatter", 1000);

	ros::Rate loop_rate(10);
  
  	while (ros::ok()){
		// generate a new integer in [1,1000]
	    static int i=0;
		i=(i+1)%1000;
		
		custom_messages::Num msg;
		msg.num =i;
		chatter_pub.publish (msg);

		ros::spinOnce();
		loop_rate.sleep();
  	}

  	return 0;
}
