#include <ros/ros.h>
#include <dvrk_components_ros/psm.h>

#include <string>
#include <stdio.h>
#include <QObject>

void newRobotState1(std::string robotState){
  std::cout << "New State for PSM1 : " << robotState << std::endl;
}

void newRobotState2(std::string robotState){
  std::cout << "New State for PSM2 : " << robotState << std::endl;
}

int main(int argc, char **argv)
{
  //Initialize ros
  ros::init(argc, argv, "test_psm");
  ros::NodeHandle nh;

  //Initialize PSM components
  PSM psm1(nh,1);
  PSM psm2(nh,2);

  //So we can see the current states of the PSM
  QObject::connect(&psm1, &PSM::robot_state_changed, newRobotState1);
  QObject::connect(&psm2, &PSM::robot_state_changed, newRobotState2);


  //Wait here till DVRK_READY or DVRK_POSITION_GOAL_CARTESIAN
  while(ros::ok()){
    if(psm1.get_robot_state() == "DVRK_READY" && psm2.get_robot_state() == "DVRK_READY"){
      break;
    }
    if(psm1.get_robot_state() == "DVRK_POSITION_GOAL_CARTESIAN" &&
       psm2.get_robot_state() == "DVRK_POSITION_GOAL_CARTESIAN"){
      break;
    }
    ros::spinOnce();
  }

  //Set the robot state!!!
  psm1.set_robot_state("DVRK_POSITION_GOAL_CARTESIAN");
  psm2.set_robot_state("DVRK_POSITION_GOAL_CARTESIAN");

  //Wait here till DVRK_POSITION_GOAL_CARTESIAN
  while(ros::ok()){
    if(psm1.get_robot_state() == "DVRK_POSITION_GOAL_CARTESIAN" &&
       psm2.get_robot_state() == "DVRK_POSITION_GOAL_CARTESIAN"){
      break;
    }
    ros::spinOnce();
  }

  //Jaw messages:
  cisst_msgs::FloatStamped   jaw_msg1;
  cisst_msgs::FloatStamped   jaw_msg2;

  //Get current positions of the arms:
  geometry_msgs::PoseStamped pos_msg1 = psm1.get_slave_cart_pos();
  geometry_msgs::PoseStamped pos_msg2 = psm2.get_slave_cart_pos();

  int count = 0;

  //Set ros loop rate to 1Hz
  ros::Rate loop_rate(1);
  while(ros::ok()){

    //If we aren't in the correct state leave loop to make sure stuff shuts down
    if(psm1.get_robot_state() != "DVRK_POSITION_GOAL_CARTESIAN" || psm2.get_robot_state() != "DVRK_POSITION_GOAL_CARTESIAN"){
      break;
    }

    //Time stamp the messages
    jaw_msg1.header.stamp = ros::Time::now();
    jaw_msg2.header.stamp = ros::Time::now();

    pos_msg1.header.stamp = ros::Time::now();
    pos_msg2.header.stamp = ros::Time::now();

    //Edit the data being sent
    if( count % 2 == 0){
       jaw_msg1.data = 0.9;
       jaw_msg2.data = 0.1;

       pos_msg1.pose.position.x = pos_msg1.pose.position.x + 0.03;
       pos_msg2.pose.position.x = pos_msg2.pose.position.x - 0.03;
    }
    else{
       jaw_msg1.data = 0.1;
       jaw_msg2.data = 0.9;

       pos_msg1.pose.position.x = pos_msg1.pose.position.x - 0.03;
       pos_msg2.pose.position.x = pos_msg2.pose.position.x + 0.03;
    }

    //Send out the data
    psm1.set_pos_goal_cart(pos_msg1);
    psm1.set_slave_jaw(jaw_msg1);

    psm2.set_pos_goal_cart(pos_msg2);
    psm2.set_slave_jaw(jaw_msg2);

    count++;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
