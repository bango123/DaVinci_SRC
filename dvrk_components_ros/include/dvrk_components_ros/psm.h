#ifndef PSM_H
#define PSM_H

#include <QObject>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <cisst_msgs/FloatStamped.h>
#include <std_msgs/String.h>

class PSM : public QObject
{
  Q_OBJECT
public:
  //Choose PSM 1(R) or 2(L)
  explicit PSM(ros::NodeHandle nh, int psm, QObject *parent = nullptr);

  //Set functions
  void set_robot_state(std::string robotState);

  //For these functions make sure the robot state is DVRK_POSITION_CARTESIAN
  void set_master_jaw     (cisst_msgs   ::FloatStamped  master_jaw);
  void set_master_cart_pos(geometry_msgs::PoseStamped   master_cart_pos);
  void set_slave_jaw      (cisst_msgs   ::FloatStamped  slave_jaw);
  void set_slave_cart_pos (geometry_msgs::PoseStamped   slave_cart_pos);

  //This goes straight to the PSM arm. NOT effected by delay channel
  void set_joint_angles   (sensor_msgs  ::JointState    joint_angles);
  //Goes straight to PSM arm too. Make srue robot state is DVRK_POSITION_GOAL_CARTESIAN
  //To control the jaw in this mode, use set_slave_jaw()
  void set_pos_goal_cart(geometry_msgs::PoseStamped  cart_pos);
  //Goes straight to PSM arm too. Make srue robot state is DVRK_POSITION_GOAL_JOINT
  //To control the jaw in this mode, use set_slave_jaw()
  void set_joint_angles_goal(sensor_msgs  ::JointState    joint_angles);

  //Get functions
  std::string get_robot_state();
  //Robot state can be:
  //  DVRK_UNINITIALIZED
  //  DVRK_HOMING_BIAS_ENCODER
  //  DVRK_HOMING_POWERING
  //  DVRK_ARM_CALIBRATED
  //  DVRK_READY (DO NOT DO ANYTHING TILL YOU HAVE REACHED AT LEAST THIS STATE. THIS MEANS STUFF IS READY!!)
  //  DVRK_POSITION_JOINT (use for set_joint_angles)
  //  DVRK_POSITION_GOAL_JOINT
  //  DVRK_POSITION_CARTESIAN (use for all of hte set_master /set_slave functions)
  //  DVRK_POSITION_GOAL_CARTESIAN (use for set_pos_goal_cart)
  //  DVRK_EFFORT_CARTESIAN
  //  DVRK_MANUAL (this state occurs when someone pushes the buttons on the robot to move them around)

  //target values refer to where the surgeon/operator is targetting the end effector
  cisst_msgs   ::FloatStamped    get_target_master_jaw();
  geometry_msgs::PoseStamped     get_target_master_cart_pos();
  cisst_msgs   ::FloatStamped    get_target_slave_jaw();
  geometry_msgs::PoseStamped     get_target_slave_cart_pos();

  //actual readings from the da vinci through readings from joint angles
  //Use Joint angles to get jaw position
  sensor_msgs  ::JointState      get_master_joint_angles();
  geometry_msgs::PoseStamped     get_master_cart_pos();
  sensor_msgs  ::JointState      get_slave_joint_angles();
  geometry_msgs::PoseStamped     get_slave_cart_pos();

//Signals that can be connected to!! Can be useful if you want to use stuff more as a notification/callback in your code.
//These signals are emitted after the callback from the corresponding subscribed topic occurs.
signals:
  void robot_state_changed(std::string robot_state);

  //Target here refers to the signal given from the master arms about where to target the slave arms
  void master_target_jaw_changed      (cisst_msgs   ::FloatStamped  master_target_jaw);
  void master_target_cart_pos_changed (geometry_msgs::PoseStamped   master_target_cart_pos);
  void slave_target_jaw_changed       (cisst_msgs   ::FloatStamped  slave_target_jaw);
  void slave_target_cart_pos_changed  (geometry_msgs::PoseStamped   slave_target_cart_pos);

  //This refers to the actual position/orientation reading from the slave arms.
  //The joint angles contain the joint angle of the jaw!!!
  void master_joint_angles_changed    (sensor_msgs  ::JointState    master_joint_angles);
  void master_cart_pos_changed        (geometry_msgs::PoseStamped   master_cart_pos);
  void slave_joint_angles_changed     (sensor_msgs  ::JointState    slave_joint_angles);
  void slave_cart_pos_changed         (geometry_msgs::PoseStamped   slave_cart_pos);

private:
  ros::NodeHandle m_nh;
  int m_psm;

  //All the subscribers
  ros::Subscriber m_sub_master_target_jaw;
  ros::Subscriber m_sub_master_target_cart_pos;
  ros::Subscriber m_sub_slave_target_jaw;
  ros::Subscriber m_sub_slave_target_cart_pos;

  ros::Subscriber m_sub_master_joint_angles;
  ros::Subscriber m_sub_master_cart_pos;
  ros::Subscriber m_sub_slave_joint_angles;
  ros::Subscriber m_sub_slave_cart_pos;

  ros::Subscriber m_sub_robot_state;

  void master_target_jawPosCallback (const cisst_msgs   ::FloatStamped& msg);
  void master_target_cartPosCallback(const geometry_msgs::PoseStamped& msg);
  void slave_target_jawPosCallback  (const cisst_msgs   ::FloatStamped& msg);
  void slave_target_cartPosCallback (const geometry_msgs::PoseStamped& msg);

  void master_jointAnglesCallback   (const sensor_msgs  ::JointState& msg);
  void master_cartPosCallback       (const geometry_msgs::PoseStamped &msg);
  void slave_jointAnglesCallback    (const sensor_msgs  ::JointState& msg);
  void slave_cartPosCallback        (const geometry_msgs::PoseStamped &msg);

  void robot_stateCallback          (const std_msgs     ::String& msg);


  std::string m_robot_state;


  cisst_msgs   ::FloatStamped  m_master_target_jaw;
  geometry_msgs::PoseStamped   m_master_target_cart_pos;
  cisst_msgs   ::FloatStamped  m_slave_target_jaw;
  geometry_msgs::PoseStamped   m_slave_target_cart_pos;

  sensor_msgs  ::JointState    m_master_joint_angles;
  geometry_msgs::PoseStamped   m_master_cart_pos;
  sensor_msgs  ::JointState    m_slave_joint_angles;
  geometry_msgs::PoseStamped   m_slave_cart_pos;

  //All the publishers
  //Use these to publish to the PSM arm!
  ros::Publisher m_pub_set_robot_state;

  ros::Publisher m_pub_master_jaw;
  ros::Publisher m_pub_master_cart_pos;
  ros::Publisher m_pub_slave_jaw;
  ros::Publisher m_pub_slave_cart_pos;

  ros::Publisher m_pub_set_goal_position;
  ros::Publisher m_pub_joint_angles_goal;

  //Untested...
  ros::Publisher m_pub_joint_angles;

};

#endif // PSM_H
