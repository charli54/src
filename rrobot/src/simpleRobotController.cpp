#include "simpleRobotController.hpp"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"

#include <ros/callback_queue.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_iface_node");
  ros::NodeHandle nh;

  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);
  
  ros::Time ts = ros::Time::now();
  
  ros::Rate rate(50);

  ros::spin();
  
  while (ros::ok())
  {
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
    robot.read();
    cm.update(ts, d);
    robot.write();
    rate.sleep();
  }

  return 0;
}