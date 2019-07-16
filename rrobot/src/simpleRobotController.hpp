#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <iostream>
class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot()
 {
   
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("JointA", &pos[0], &vel[0], &eff[0]); //pos vel eff outputs of the state message...
    jnt_state_interface.registerHandle(state_handle_a);
    hardware_interface::JointStateHandle state_handle_b("JointB", &pos[1], &vel[1], &eff[1]); //pos vel eff outputs of the state message...
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface

    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("JointA"), &cmd[0]); //cmd is the commanded value depending on the controller.
    jnt_pos_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("JointB"), &cmd[1]); //cmd is the commanded value depending on the controller.
    jnt_pos_interface.registerHandle(pos_handle_b);

    registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_vel_interface);
jnt_vel_interface.registerHandle(pos_handle_a);
    jnt_vel_interface.registerHandle(pos_handle_b);
   
}

  virtual ~MyRobot()
  {}

  void write()
  {
	 // std::cout<<"write "<<"  "<<cmd[0]<<" "<<cmd[1]<<std::endl;
   // std::cout<<"velocity "<<"  "<<vel[0]<<" "<<vel[1]<<std::endl;
  }

  void read()
  {
     //     std::cout<<"read "<<"  "<<pos[0]<<" "<<pos[1]<<std::endl;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};