#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv){
	
	//The third argument passing to the init() function is the name of the node 
	ros::init(argc,argv,"talker");

	//create an acces point to communicate with ROS
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()){
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello World " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

return 0;
}