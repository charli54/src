#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//This file will send message over a topic

int main(int argc, char **argv)
{
	//La fonction init() a besoin de voire argc et argv pour pouvoir les remapper
	//le dernier argument est le nom du node
	ros::init(argc, argv, "talker");


	//NodeHandle est le point d'acces avec ROS
	ros::NodeHandle n;


	//initialise un publisher qui va comminiquer avec le topic "chatter"

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);

	//Fonction qui va composer le message
	int count = 0;
	while (ros::ok()){
		//cree un objet message
		std_msgs::String msg;

		//fabrique le message
		std::stringstream ss;
		ss << "hello world " << count;
  		msg.data = ss.str();

  		ROS_INFO("%s", msg.data.c_str());

  		//On publie le message
  		chatter_pub.publish(msg);

  		ros::spinOnce();

  		loop_rate.sleep();
  		count++;
	}
	return 0;
}