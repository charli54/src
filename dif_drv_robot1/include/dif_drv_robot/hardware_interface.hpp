#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
 { 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("jointA", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("jointB", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("jointA"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_a);

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("jointB"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_b);

   registerInterface(&jnt_pos_interface);
  }


ros::Time getTime() const {return ros::Time::now();}
ros::Duration getPeriod() const {return ros::Duration(0.01);}

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};