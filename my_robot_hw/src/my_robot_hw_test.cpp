#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <my_robot_hw/my_robot_hw.h>

using namespace my_robot_hw_if;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  MyRobotHW hw;

  ros::NodeHandle nh;
  controller_manager::ControllerManager cm(&hw, nh);

  ros::Duration period(1.0);
  while (ros::ok())
  {
    ROS_INFO("loop");
    hw.read();
    cm.update(ros::Time::now(), period);
    hw.write();
    period.sleep();
  }
}

