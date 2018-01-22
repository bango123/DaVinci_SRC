#include <ros/ros.h>
#include <dvrk_components_ros/psm.h>

#include <string>
#include <stdio.h>
#include <vector>
#include <QObject>
#include <math.h>

#include<image_transport/image_transport.h>
#include<image_transport/camera_subscriber.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//Image matrices
cv::Mat img_L;
cv::Mat img_R;

void newRobotState1(std::string robotState){
  std::cout << "New State for PSM1 : " << robotState << std::endl;
}

void newRobotState2(std::string robotState){
  std::cout << "New State for PSM2 : " << robotState << std::endl;
}

void imageCallback_L(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
  img_L = cv_ptr->image;

  cv::cvtColor(img_L, img_L, cv::COLOR_BGR2RGB);
}

void imageCallback_R(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
  img_R = cv_ptr->image;

  cv::cvtColor(img_R, img_R, cv::COLOR_BGR2RGB);
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


  //Wait here till DVRK_READY or DVRK_POSITION_GOAL_JOINT
  while(ros::ok()){
    if(psm1.get_robot_state() == "DVRK_READY" && psm2.get_robot_state() == "DVRK_READY"){
      break;
    }
    if(psm1.get_robot_state() == "DVRK_POSITION_GOAL_JOINT" &&
       psm2.get_robot_state() == "DVRK_POSITION_GOAL_JOINT"){
      break;
    }
    ros::spinOnce();
  }

  if(!ros::ok()){
    return 0;
  }

  //Set the robot state!!!
  psm1.set_robot_state("DVRK_POSITION_GOAL_JOINT");
  psm2.set_robot_state("DVRK_POSITION_GOAL_JOINT");

  //Wait here till DVRK_POSITION_GOAL_JOINT
  while(ros::ok()){
    if(psm1.get_robot_state() == "DVRK_POSITION_GOAL_JOINT" &&
       psm2.get_robot_state() == "DVRK_POSITION_GOAL_JOINT"){
      break;
    }
    ros::spinOnce();
  }

  if(!ros::ok()){
    return 0;
  }

  sensor_msgs::JointState psm1_joints_msg = psm1.get_slave_joint_angles();

  //Set the outer_roll, outer_wrist_pitch, outer_wrist_pitch, and jaw to 0
  psm1_joints_msg.header.stamp = ros::Time::now();
  psm1_joints_msg.position[3] = 0.0;
  psm1_joints_msg.position[4] = 0.0;
  psm1_joints_msg.position[5] = 0.0;
  psm1_joints_msg.position[6] = 0.0;

  psm1.set_joint_angles_goal(psm1_joints_msg);


  //Set up subscribers for the images now that we are in position :D
  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub_L = it.subscribe("stereo/left/image_raw"  , 1, imageCallback_L);
  image_transport::Subscriber sub_R = it.subscribe("stereo/right/image_raw" , 1, imageCallback_R);

  //Look for images with a checkerbooard!!!

  while(ros::ok()){

    if(!img_L.empty() && !img_R.empty()){

      std::vector<cv::Point2f> corners_L;
      bool patterFound_L = cv::findChessboardCorners(img_L, cv::Size(8,6), corners_L);
      std::cout << patterFound_L << std::endl;

      if(patterFound_L){
        cv::drawChessboardCorners(img_L, cv::Size(8,6),corners_L, patterFound_L);
      }
      cv::imshow("Left Checkerboard", img_L);
      cv::waitKey(1000);

    }
    ros::spinOnce();
  }


  return 0;
}
