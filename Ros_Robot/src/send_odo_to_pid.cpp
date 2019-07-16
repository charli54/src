#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

double R_spd = 0.0;
double L_spd = 0.0;

int publishData = 0;

void publish_spd(const ros::TimerEvent &event){
//	right_wheel_pub.publish(R_spd);
}

void listen_to_odo(const geometry_msgs::Twist::ConstPtr& msg){
	//Store wheels velocity inside golbal variables
    R_spd = msg->linear.x;
    L_spd = msg->linear.y;

    //Rise a flag to tell that message has been received 
    publishData = 1;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "send_odo_to_pid");

	ros::NodeHandle n;
	
	//Subscriber to /ard_odo
	ros::Subscriber sub = n.subscribe("ard_odo", 1000, listen_to_odo);
	
	//Two publisher to publish the state of the wheels velocity ti PID
	ros::Publisher right_wheel_pub = n.advertise<std_msgs::Float64>("right_wheel_spd", 50);
	//ros::Publisher left_wheel_pub = n.advertise<std_msgs::Float64>("left_wheel_spd", 50);
  	
  	double dataTest;
  	
    while(n.ok()){

		ros::spinOnce(); 

		if(publishData){
		    ros::Time current_time = ros::Time::now();

		    nav_msgs::Odometry dataTest;
		    
		    dataTest.header.stamp = current_time;
		    //dataTest.twist.twist.linear.x = data;
		    right_wheel_pub.publish(dataTest);
		    publishData = 0;
		}
    }

	return 0;
}