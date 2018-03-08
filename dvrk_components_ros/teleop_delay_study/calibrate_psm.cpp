#include <ros/ros.h>
#include <dvrk_components_ros/psm.h>
#include <dvrk_components_ros/teleop.h>

#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

const double PI   = 3.141592653589793238463;
const double PI_2 = 1.5708;

Eigen::Matrix4f getForwardKin(double a, double alpha, double d, double theta){

    double ca = cos(alpha);
    double sa = sin(alpha);

    double ct = cos(theta);
    double st = sin(theta);

    Eigen::Matrix4f transform;
    transform << ct, -st*ca,  st*sa ,  a*ct,
                 st,  ct*ca, -ct*sa ,  a*st,
                  0,     sa,     ca ,  d,
                  0,    0  ,     0  ,  1;

    return transform;

}

void testKin(sensor_msgs::JointState joint_angles, geometry_msgs::PoseStamped cart_pos){

    Eigen::Matrix4f T;

    //Outer Yaw
    T =    getForwardKin(0.0, PI_2,
                         0.0, joint_angles.position[0] + PI_2);

    //Outer Pitch
    T =    getForwardKin(0.0, -PI_2,
                         0.0, joint_angles.position[1] - PI_2) * T;

    //Insertion
    T =    getForwardKin(0.0,  PI_2,
                        joint_angles.position[2] - 0.4318, 0.0) * T;

    //Outer Roll
    T =   getForwardKin(0.0, 0.0,
                        0.4162, joint_angles.position[3]) * T;

    //Wrist Pitch
    T =    getForwardKin(0.0,  -PI_2,
                         0.0, joint_angles.position[4] - PI_2) * T;
    //Wrist Yaw
    T =   getForwardKin( 0.0091, -PI_2,
                         0.0, joint_angles.position[5] - PI_2) * T;

    //End Effector
    T =    getForwardKin( 0.0, -PI_2,
                          0.0102,   0.0) * T;


    float px = cart_pos.pose.position.x;
    float py = cart_pos.pose.position.y;
    float pz = cart_pos.pose.position.z;

    float qw = cart_pos.pose.orientation.w;
    float qx = cart_pos.pose.orientation.x;
    float qy = cart_pos.pose.orientation.y;
    float qz = cart_pos.pose.orientation.z;

    Eigen::Transform<float,3,Eigen::Affine> T_position;
    T_position = Eigen::Quaternion<float>(qw, qx, qy, qz) *  Eigen::Translation3f(px, py, pz);
    std::cout << "T_Position = " << std::endl;
    std::cout << T_position.matrix() << std::endl;

    std::cout << "Transform = " << std::endl;
    std::cout << T << std::endl;

//     Eigen::Matrix4f T_position;
//     T_position <<
//                    1.0f - 2.0f*qy*qy - 2.0f*qz*qz, 2.0f*qx*qy - 2.0f*qz*qw, 2.0f*qx*qz + 2.0f*qy*qw, px,
//                    2.0f*qx*qy + 2.0f*qz*qw, 1.0f - 2.0f*qx*qx - 2.0f*qz*qz, 2.0f*qy*qz - 2.0f*qx*qw, py,
//                    2.0f*qx*qz - 2.0f*qy*qw, 2.0f*qy*qz + 2.0f*qx*qw, 1.0f - 2.0f*qx*qx - 2.0f*qy*qy, pz,
//                    0.0f, 0.0f, 0.0f, 1.0f;

     Eigen::Matrix4f T_base = T.inverse()*T_position.matrix();
     std::cout << "Base = " << std::endl;
     std::cout << T_base    << std::endl;


}

Eigen::Vector4f getEndEffectorPos(sensor_msgs::JointState  joint_angles, geometry_msgs::PoseStamped cart_pos){
    //End Effector
     Eigen::Matrix4f T;

    T =    getForwardKin( 0.0, -PI_2,
                          0.0102,   0.0);

    float px = cart_pos.pose.position.x;
    float py = cart_pos.pose.position.y;
    float pz = cart_pos.pose.position.z;

    Eigen::Vector4f p(px, py, pz, 1);
    return T*p;

//    float qw = cart_pos.pose.orientation.w;
//    float qx = cart_pos.pose.orientation.x;
//    float qy = cart_pos.pose.orientation.y;
//    float qz = cart_pos.pose.orientation.z;


//    Eigen::Transform<float,3,Eigen::Affine> T_position;
//    T_position = Eigen::Quaternion<float>(qw, qx, qy, qz) *  Eigen::Translation3f(px, py, pz);


}

int main(int argc, char **argv)
{
     std::string calFile_str;

    if(argc == 2){
        calFile_str = argv[1];
    }
    else{
        calFile_str = "/home/arclab/catkin_ws/src/dvrk_components_ros/teleop_delay_study/calFile.txt";
    }

    std::ofstream calFile;
    calFile.open(calFile_str);

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

  if(!psm1.set_robot_state_and_wait("DVRK_POSITION_GOAL_JOINT")){
    return 0;
  }
  std::cout << "---   Setting PSM2  ---" << std::endl;

  if(!psm2.set_robot_state_and_wait("DVRK_POSITION_GOAL_JOINT")){
      return 0;
    }

  std::cout << "---   Zeroing Wrist   ---" << std::endl;

  sensor_msgs  ::JointState psm1_angles = psm1.get_master_joint_angles();
  sensor_msgs  ::JointState psm2_angles = psm2.get_master_joint_angles();

  psm1_angles.position[4] = 0.0;
  psm1_angles.position[5] = 0.0;
  psm1_angles.position[6] = 0.0;

  psm2_angles.position[4] = 0.0;
  psm2_angles.position[5] = 0.0;
  psm2_angles.position[6] = 0.0;

  psm1.set_joint_angles_goal(psm1_angles);
  psm2.set_joint_angles_goal(psm2_angles);


  char input;

  std::cout << "Pick peg points by manual operation with PSM1: " << std::endl;
  calFile << "PSM1:\n";

  //need to touch the 4 posts
  for(int i = 0; i < 4; i++){
        std::cin >> input;

        ros::spinOnce();

        //Eigen::Vector4f pos(getEndEffectorPos(psm1.get_slave_joint_angles(), psm1.get_slave_cart_pos()));
        geometry_msgs::PoseStamped pos = psm1.get_slave_cart_pos();

        std::cout << std::endl << "New point: " << std::endl << pos << std::endl;

        calFile << pos.pose.position.x << "," << pos.pose.position.y << "," << pos.pose.position.z << std::endl;

  }

  std::cout << "Pick peg points by manual operation with PSM2: " << std::endl;
  calFile << "PSM2:\n";

  //need to touch the 4 posts
  for(int i = 0; i < 4; i++){
        std::cin >> input;

        ros::spinOnce();

        geometry_msgs::PoseStamped pos = psm2.get_slave_cart_pos();

        std::cout << std::endl << "New point: " << std::endl << pos << std::endl;

        calFile << pos.pose.position.x << "," << pos.pose.position.y << "," << pos.pose.position.z << std::endl;

  }


  calFile.close();
  return 0;
}
