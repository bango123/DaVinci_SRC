#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dvrk_components_ros/psm.h>

#include <QObject>
#include <string>
#include <stdio.h>

bool operatorPresent;

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

void coag_callback(const sensor_msgs::Joy& coag_msg){
   if( 1 == coag_msg.buttons[0] ){
      operatorPresent = !operatorPresent;
   }

   if(operatorPresent){
     std::cout << "Operator Present" << std::endl;
   }
   else{
     std::cout << "Operator Not Present" << std::endl;
   }
}

void connectArm(const PSM &psm){
  QObject::connect(&psm, &PSM::master_target_jaw_changed,
                   &psm, &PSM::set_master_jaw);

  QObject::connect(&psm, &PSM::slave_target_jaw_changed,
                   &psm, &PSM::set_slave_jaw);

  QObject::connect(&psm, &PSM::master_target_cart_pos_changed,
                   &psm, &PSM::set_master_cart_pos);

  QObject::connect(&psm, &PSM::slave_target_cart_pos_changed,
                   &psm, &PSM::set_slave_cart_pos);
}

void disconnectArm(const PSM &psm){
  QObject::disconnect(&psm, &PSM::master_target_jaw_changed,
                      &psm, &PSM::set_master_jaw);

  QObject::disconnect(&psm, &PSM::slave_target_jaw_changed,
                      &psm, &PSM::set_slave_jaw);

  QObject::disconnect(&psm, &PSM::master_target_cart_pos_changed,
                      &psm, &PSM::set_master_cart_pos);

  QObject::disconnect(&psm, &PSM::slave_target_cart_pos_changed,
                      &psm, &PSM::set_slave_cart_pos);
}

int main(int argc, char **argv)
{
  //Initialize ros
  ros::init(argc, argv, "test_psm");
  ros::NodeHandle nh;

  operatorPresent = false;

  //Subscribed to the coag footpedal
  ros::Subscriber coag_sub = nh.subscribe("/dvrk/footpedals/coag", 1, coag_callback);

  //Initialize PSM components
  PSM psm1(nh,1);
  PSM psm2(nh,2);

  //Connect signals from delay, scale, and robot state changes
  QObject::connect(&psm1, &PSM::delay_changed, newDelay);
  QObject::connect(&psm1, &PSM::scale_changed, newScale);
  QObject::connect(&psm1, &PSM::robot_state_changed, newRobotState1);
  QObject::connect(&psm2, &PSM::robot_state_changed, newRobotState2);



  while(ros::ok()){

    if(operatorPresent){
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
