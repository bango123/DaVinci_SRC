#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cisst_msgs/FloatStamped.h>
#include <sensor_msgs/Joy.h>
#include <iostream>

//Kept globally that way we can use them in the callbacks
//BAD PRACTICE!!!! :(
ros::Publisher psm1_jaw_pub;
ros::Publisher psm1_cart_pub;

ros::Publisher psm2_jaw_pub;
ros::Publisher psm2_cart_pub;

bool operatorPresent;

//Jaw and position callbacks (occur when message has arrived)
void psm1_jaw_callback(const cisst_msgs::FloatStamped& jaw_msg){
  if( !operatorPresent){
    return;
  }
  psm1_jaw_pub.publish(jaw_msg);
}

void psm1_cart_callback(const geometry_msgs::PoseStamped& cart_msg){
  if( !operatorPresent){
    return;
  }
  psm1_cart_pub.publish(cart_msg);
}

void psm2_jaw_callback(const cisst_msgs::FloatStamped& jaw_msg){
  if( !operatorPresent){
    return;
  }
  psm2_jaw_pub.publish(jaw_msg);
}

void psm2_cart_callback(const geometry_msgs::PoseStamped& cart_msg){
  if( !operatorPresent){
    return;
  }
  psm2_cart_pub.publish(cart_msg);
}

void coag_callback(const sensor_msgs::Joy& coag_msg){
   if( 1 == coag_msg.buttons[0] ){
      operatorPresent = !operatorPresent;
   }
}

int main(int argc, char **argv)
{
  operatorPresent = false;

  if(argc != 2){
     std::cout << "Please specify 'master' or 'slave'" << std::endl;
     return 0;
  }
  std::string side_param = argv[1];

  //Need to create seperate node for master/slave thing
  if( !side_param.compare("master") ){
      ros::init(argc, argv, "master_passThrough");
  }
  else{
      ros::init(argc, argv, "slave_passThrough");
  }


  ros::NodeHandle nh;

  //All of the subscribers used
  ros::Subscriber psm1_jaw_sub;
  ros::Subscriber psm1_cart_sub;

  ros::Subscriber psm2_jaw_sub;
  ros::Subscriber psm2_cart_sub;

  ros::Subscriber coag_sub = nh.subscribe("/dvrk/footpedals/coag", 1, coag_callback);

  //See if passThrough is for master or slave side.
  //Subscribe to the appropriate topic and publish to the appropriate one too
  if( !side_param.compare("master") ){
    psm1_jaw_pub  = nh.advertise <cisst_msgs::FloatStamped>   ("dvrk/PSM1/master/set_jaw_position" , 1);
    psm1_cart_pub = nh.advertise <geometry_msgs::PoseStamped> ("dvrk/PSM1/master/set_position_cartesian" , 1);

    psm2_jaw_pub  = nh.advertise <cisst_msgs::FloatStamped>   ("dvrk/PSM2/master/set_jaw_position" , 1);
    psm2_cart_pub = nh.advertise <geometry_msgs::PoseStamped> ("dvrk/PSM2/master/set_position_cartesian" , 1);

    psm1_jaw_sub  = nh.subscribe("dvrk/PSM1/master/target_jaw_position"       , 1, psm1_jaw_callback);
    psm1_cart_sub = nh.subscribe("dvrk/PSM1/master/target_position_cartesian" , 1, psm1_cart_callback);

    psm2_jaw_sub  = nh.subscribe("dvrk/PSM2/master/target_jaw_position"       , 1, psm2_jaw_callback);
    psm2_cart_sub = nh.subscribe("dvrk/PSM2/master/target_position_cartesian" , 1, psm2_cart_callback);
  }
  else if( !side_param.compare("slave")){
    psm1_jaw_pub  = nh.advertise <cisst_msgs::FloatStamped>   ("dvrk/PSM1/slave/set_jaw_position" , 1);
    psm1_cart_pub = nh.advertise <geometry_msgs::PoseStamped> ("dvrk/PSM1/slave/set_position_cartesian" , 1);

    psm2_jaw_pub  = nh.advertise <cisst_msgs::FloatStamped>   ("dvrk/PSM2/slave/set_jaw_position" , 1);
    psm2_cart_pub = nh.advertise <geometry_msgs::PoseStamped> ("dvrk/PSM2/slave/set_position_cartesian" , 1);

    psm1_jaw_sub  = nh.subscribe("dvrk/PSM1/slave/target_jaw_position"       , 1, psm1_jaw_callback);
    psm1_cart_sub = nh.subscribe("dvrk/PSM1/slave/target_position_cartesian" , 1, psm1_cart_callback);

    psm2_jaw_sub  = nh.subscribe("dvrk/PSM2/slave/target_jaw_position"       , 1, psm2_jaw_callback);
    psm2_cart_sub = nh.subscribe("dvrk/PSM2/slave/target_position_cartesian" , 1, psm2_cart_callback);
  }
  else {
    std::cout << "Please specify 'master' or 'slave'" <<std::endl;
    return 0;
  }

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
