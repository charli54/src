#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>





int main(int argc, char** argv){

	ros::init(argc, argv, "robot_tf_listener");
	ros::NodeHandle n;

	//Create a tranformListener object
	tf::TransformListener listener;

}