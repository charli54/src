#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <iostream>

using namespace std;

int publishData = 0;

double data = 0.0;

void printData(const geometry_msgs::Twist::ConstPtr& twist_msg){
    cout << "Speed data:" << twist_msg->linear.x << "\n";
    ROS_INFO("I heard: [%s]", twist_msg->linear.x);
    data = twist_msg->linear.x;
    publishData = 1;
    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ard_odo", 1000, printData);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    while(n.ok()){
	ros::spinOnce(); 

	if(publishData){
	    ros::Time current_time = ros::Time::now();

	    nav_msgs::Odometry dataTest;
	    
	    dataTest.header.stamp = current_time;
	    dataTest.twist.twist.linear.x = data;
	    odom_pub.publish(dataTest);
	    publishData = 0;
		}
    }
    return 0;
}
