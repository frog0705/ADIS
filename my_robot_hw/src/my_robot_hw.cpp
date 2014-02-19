#include <my_robot_hw/my_robot_hw.h>

namespace my_robot_hw_if
{

MyRobotHW::MyRobotHW()
{
  using namespace hardware_interface;

  // Initialize raw data
  joint_position_.resize(2);
  joint_velocity_.resize(2);
  joint_effort_.resize(2);
  joint_effort_command_.resize(2);
  joint_velocity_command_.resize(2);
  joint_name_.resize(2);

  joint_name_[0] = "joint1";
  joint_position_[0] = 1.0;
  joint_velocity_[0] = 0.0;
  joint_effort_[0] = 0.1;
  joint_effort_command_[0] = 0.0;
  joint_velocity_command_[0] = 0.0;

  joint_name_[1] = "joint2";
  joint_position_[1] = 1.0;
  joint_velocity_[1] = 0.0;
  joint_effort_[1] = 0.1;
  joint_effort_command_[1] = 0.0;
  joint_velocity_command_[1] = 0.0;

  // Populate hardware interfaces
  js_interface_.registerHandle(JointStateHandle(joint_name_[0], &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]));
  js_interface_.registerHandle(JointStateHandle(joint_name_[1], &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]));

  ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[0]), &joint_effort_command_[0]));
  ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[1]), &joint_effort_command_[1]));

  vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[0]), &joint_velocity_command_[0]));
  vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[1]), &joint_velocity_command_[1]));

  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&vj_interface_);
}


void MyRobotHW::read()
{

}

void MyRobotHW::write()
{
}

}
