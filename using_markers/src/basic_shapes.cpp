#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub=n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	uint32_t shape = visualization_msgs::Marker::CUBE;

	while(ros::ok())
	{
		visualization_msgs::Marker marker;

		marker.header.frame_id = "/my_frame";				//name space
		marker.header.stamp = ros::Time::now();

		marker.ns = "basic_shapes";
		marker.id = 0;

		marker.type = shape;

		marker.action = visualization_msgs::Marker::ADD;	//http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html

		//Postion et orientation de l'objet
		marker.pose.position.x = 0;							//geometry_msgs/Pose pose 
		marker.pose.position.y = 0;							//Les message est composé de Point position et Quaternion orientation
		marker.pose.position.z = 0;							//pose est composé de 3 float64 (x, y, z)
		marker.pose.orientation.x = 0.0;					//orientation est composé de 4 float64 (x, y, z, w)
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//Echelle de l'objet
		marker.scale.x = 1.0;								//geometry_msgs/Vector3 scale composé de 3 float64 (x, y, z)
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;

		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

	//Tant que personne ne souscrit on bloque
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

	marker_pub.publish(marker);
	
	    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

    r.sleep();


	}
	return 0;
}