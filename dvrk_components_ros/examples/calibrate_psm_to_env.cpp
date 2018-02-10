#include <ros/ros.h>
#include <dvrk_components_ros/psm.h>
#include <dvrk_components_ros/teleop.h>

#include <string>
#include <stdio.h>
#include <vector>
#include <QObject>
#include <math.h>


int main(int argc, char **argv)
{
  //Initialize ros
  ros::init(argc, argv, "test_psm");
  ros::NodeHandle nh;

  //Initialize teleop component
  Teleop teleop(nh);

  //Initialize PSM components
  PSM psm1(nh,1);
  PSM psm2(nh,2);

  std::cout << "---  Reseting DVRK  ---" << std::endl;
  if(!teleop.reset_dvrk())
  {
    return 0;
  }
  std::cout << "---Turning on System---" << std::endl;

  if(!teleop.turn_onTeleop()){
    return 0;
  }
  std::cout << "--- Turning on Ros  ---" << std::endl;

  if(!teleop.set_ros_only_wait(true)){
    return 0;
  }

  std::cout << "---   Setting PSM1  ---" << std::endl;

  if(!psm1.set_robot_state_and_wait("DVRK_POSITION_GOAL_CARTESIAN")){
    return 0;
  }
  std::cout << "---   Setting PSM2  ---" << std::endl;

  if(!psm2.set_robot_state_and_wait("DVRK_POSITION_GOAL_CARTESIAN")){
    return 0;
  }

  std::cout << "---   Zeroing Jaw   ---" << std::endl;

  std::cout << "Current Jaw Angles PSM1: " << psm1.get_slave_jaw() << std::endl;
  std::cout << "Current Jaw Angles PSM2: " << psm2.get_slave_jaw() << std::endl;


  return 0;
}
