#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dvrk_components_ros/psm.h>
#include <dvrk_components_ros/teleop.h>

#include <QObject>
#include <string>
#include <stdio.h>


//All of these functions are slots for the signals emitted from the teleop and PSM components.
//This is done as a test

void newDelay(float delay){
  std::cout << "New Delay : " << delay << std::endl;
}

void newScale(float scale){
  std::cout << "New Scale : " << scale << std::endl;
}

void newRobotState1(std::string robotState){
  std::cout << "New State for PSM1 : " << robotState << std::endl;
}

void newRobotState2(std::string robotState){
  std::cout << "New State for PSM2 : " << robotState << std::endl;
}

void clutch_footpedal(bool pressed){
  std::cout << "Clutch is : ";

  if(pressed){
    std::cout << "PRESSED" << std::endl;
  }
  else{
    std::cout << "RELEASED" << std::endl;
  }
}

void camera_footpedal(bool pressed){
  std::cout << "Camera is : ";

  if(pressed){
    std::cout << "PRESSED" << std::endl;
  }
  else{
    std::cout << "RELEASED" << std::endl;
  }
}

void coag_footpedal(bool pressed){
  std::cout << "Coag is : ";

  if(pressed){
    std::cout << "PRESSED" << std::endl;
  }
  else{
    std::cout << "RELEASED" << std::endl;
  }
}

void cameraPlus_footpedal(bool pressed){
  std::cout << "Camera Plus is : ";

  if(pressed){
    std::cout << "PRESSED" << std::endl;
  }
  else{
    std::cout << "RELEASED" << std::endl;
  }
}

void cameraMinus_footpedal(bool pressed){
  std::cout << "Camera Minu is : ";

  if(pressed){
    std::cout << "PRESSED" << std::endl;
  }
  else{
    std::cout << "RELEASED" << std::endl;
  }
}

void operatorPresent_callback(bool operatorPresent){
  std::cout << "Operator is: ";
  if(operatorPresent){
    std::cout << "PRESENT" << std::endl;
  }
  else{
    std::cout << "ABSENT" << std::endl;
  }
}

void powerStatus_callback(bool powerStatus){
  std::cout << "Power is :";
  if(powerStatus){
    std::cout << "ON" <<std::endl;
  }
  else{
    std::cout << "OFF" << std::endl;
  }
}

void rosOnly_callback(bool powerStatus){
  std::cout << "Ros Only is :";
  if(powerStatus){
    std::cout << "ON" <<std::endl;
  }
  else{
    std::cout << "OFF" << std::endl;
  }
}

void teleopStatus_callback(bool powerStatus){
  std::cout << "Teleop is :";
  if(powerStatus){
    std::cout << "ON" <<std::endl;
  }
  else{
    std::cout << "OFF" << std::endl;
  }
}

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

  //Connect signals from delay, scale, and robot state changes
  QObject::connect(&teleop, &Teleop::delay_changed, newDelay);
  QObject::connect(&teleop, &Teleop::scale_changed, newScale);

  QObject::connect(&psm1, &PSM::robot_state_changed, newRobotState1);
  QObject::connect(&psm2, &PSM::robot_state_changed, newRobotState2);

  //connect footpedal signals (just to verify :D)
  QObject::connect(&teleop, &Teleop::clutch_changed,      clutch_footpedal);
  QObject::connect(&teleop, &Teleop::camera_changed,      camera_footpedal);
  QObject::connect(&teleop, &Teleop::coag_changed,        coag_footpedal);
  QObject::connect(&teleop, &Teleop::camera_changed,      camera_footpedal);
  QObject::connect(&teleop, &Teleop::cameraPlus_changed,  cameraPlus_footpedal);
  QObject::connect(&teleop, &Teleop::cameraMinus_changed, cameraMinus_footpedal);

  //connect to other signals just to verify more!!!
  QObject::connect(&teleop, &Teleop::operatorPresent_changed, operatorPresent_callback);
  QObject::connect(&teleop, &Teleop::powerStatus_changed,     powerStatus_callback);
  QObject::connect(&teleop, &Teleop::rosOnly_changed,         rosOnly_callback);
  QObject::connect(&teleop, &Teleop::teleopStatus_changed,    teleopStatus_callback);


  if(!teleop.reset_dvrk())
  {
    return 0;
  }

  if(!teleop.turn_onTeleop()){
    return 0;
  }

  if(!teleop.set_ros_only_wait(true)){
    return 0;
  }

  if(!psm1.set_robot_state_and_wait("DVRK_POSITION_CARTESIAN")){
    return 0;
  }

  if(!psm2.set_robot_state_and_wait("DVRK_POSITION_CARTESIAN")){
    return 0;
  }


  //Passthrough code
  while(ros::ok()){
    if(teleop.get_operatorPresent() && teleop.get_powerStatus() && teleop.get_teleopStatus()){
      psm1.set_master_jaw(psm1.get_target_master_jaw());
      psm1.set_master_cart_pos(psm1.get_target_master_cart_pos());
      psm1.set_slave_jaw(psm1.get_target_slave_jaw());
      psm1.set_slave_cart_pos(psm1.get_target_slave_cart_pos());

      psm2.set_master_jaw(psm2.get_target_master_jaw());
      psm2.set_master_cart_pos(psm2.get_target_master_cart_pos());
      psm2.set_slave_jaw(psm2.get_target_slave_jaw());
      psm2.set_slave_cart_pos(psm2.get_target_slave_cart_pos());
    }

    ros::spinOnce();
  }

}
