#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <geometry_msgs/Twist.h>

    double R_spd = 0.0;
    double L_spd = 0.0;

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
 { 

  pos_[0] = 0;
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("base_to_roue_gauche", &pos_[0], &vel_[0], &eff_[0]);
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("base_to_roue_droite", &pos_[1], &vel_[1], &eff_[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("base_to_roue_gauche"), &cmd_[0]);
   jnt_pos_interface.registerHandle(pos_handle_a);
   jnt_vel_interface.registerHandle(pos_handle_a);
   

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("base_to_roue_droite"), &cmd_[1]);
   jnt_pos_interface.registerHandle(pos_handle_b);
   jnt_vel_interface.registerHandle(pos_handle_b);
   
   registerInterface(&jnt_pos_interface);
   registerInterface(&jnt_vel_interface);

   odo_sub = nh.subscribe("ard_odo", 1, &MyRobot::getOdometryFromWheelEncoders, this);

  }


  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}


  void read(ros::Time t){
    //ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
  	double interval = t.toSec() - lastTime_.toSec();

    vel_[0] = wheelLinearVelocity[0];
    vel_[1] = wheelLinearVelocity[1];

    /*double v = (wheelLinearVelocity[0] + wheelLinearVelocity[1])/2;
  	double vx = v * cos(th);
  	double vy = v * sin(th);*/

    pos_[0] += (wheelLinearVelocity[0] / 0.0695) * interval;
    pos_[1] += (wheelLinearVelocity[1] / 0.0695) * interval;

    lastTime_ = ros::Time::now();
    
  }

  void write(ros::Time t){
    //Update pos_ and vel_
    //ROS_INFO_STREAM("Commands for joints: " << vel_[0] << ", " << vel_[1]);
    //double interval = t.toSec() - lastTime_.toSec();
    //double deltaRoueDroite = cmd_[0] * interval;
    //pos_[0] += deltaRoueDroite;
    //pos_[1] += cmd_[1] * interval;



    msg.linear.x = cmd_[0];
    msg.linear.y = cmd_[1];
    pub.publish(msg);
    //lastTime_ = ros::Time::now();


  }


private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  ros::Time lastTime_ = ros::Time::now();
  double cmd_[2];
  double pos_[2] = {0,0};
  double vel_[2];
  double eff_[2];

  double wheelLinearVelocity[2];

  ros::NodeHandle nh;
  geometry_msgs::Twist msg;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("ard_cmd_vel", 1000);

  ros::Subscriber odo_sub;

  void getOdometryFromWheelEncoders(const geometry_msgs::Twist& odo_msg){
  	wheelLinearVelocity[0] = odo_msg.linear.x;
  	wheelLinearVelocity[1] = odo_msg.linear.y;
  }
};