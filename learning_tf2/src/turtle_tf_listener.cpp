//###############################################################################################################
//		http://docs.ros.org/melodic/api/tf/html/c++/
//###############################################################################################################

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle node;

	//---------------ADD A SECND TURTLE --------------------------------------------------------------------------
	ros::service::waitForService("spawn");											//wait until the service is up
	ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn srv;
	add_turtle.call(srv);
	//------------------------------------------------------------------------------------------------------------

	//--------------PUBLISH VELOCITY -----------------------------------------------------------------------------
	ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

	//Create a tranformListener object
	tf::TransformListener listener;
	
	ros::Rate rate(10.0);
  	
  	while (node.ok()){
	    tf::StampedTransform transform;
		//On realise la tranformation depuis le repere /turtle1 jusqu'au repere de /turtle2 et on la stocke dans transform
		try{
	      listener.lookupTransform("/turtle2", "/carrot",ros::Time(0), transform);
	    }
	    catch (tf::TransformException &ex) {
	      ROS_ERROR("%s",ex.what());
	      ros::Duration(1.0).sleep();
	      continue;
	    }

	    geometry_msgs::Twist vel_msg;

	    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
	                                    transform.getOrigin().x());
	    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
	                                  pow(transform.getOrigin().y(), 2));
	    turtle_vel.publish(vel_msg);

	    rate.sleep();
	  }
	  return 0;
};