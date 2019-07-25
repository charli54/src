#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

const double L = 0.25		//distance entre les roues 

double R_spd = 0.0;
double L_spd = 0.0;



int publishData = 0;

void listen_to_odo(const geometry_msgs::Twist::ConstPtr& msg){
	//Store wheels velocity inside golbal variables
    R_spd = msg->linear.x;
    L_spd = msg->linear.y;


    //Rise a flag to tell that message has been received 
    publishData = 1;
}

int main(int argc, char** argv){

	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx = 0.1;
	double vy = -0.1;
	double vth = 0.1;

	ros::init(argc, argv, "Publish_odometry_from encoders");
	ros::NodeHandle n;

	//Subscriber to /ard_odo
	ros::Subscriber ard_odo_sub = n.subscribe("ard_odo", 1000, listen_to_odo);

	//Publisher pour l'odometry des encoders
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	while(n.ok()){
		if(publishData){
			ros::Time current_time = ros::Time::now();
			double dt = (current_time - last_time).toSec();
			vth = (vL + vR)/L
			v = (vL + vR)/2;
			double d_th = vth * dt

			vx = v * cos(th);
			vy = v * sin(th);



			last_time = ros::Time::now();
		}
	}



}