#include <dvrk_components_ros/psm.h>

PSM::PSM(ros::NodeHandle nh, int psm, QObject *parent) : m_nh(nh), m_psm(psm), QObject(parent)
{
    std::string ros_node_string = "/dvrk/PSM" + std::to_string(psm) + "/";

    //Subscribe to all of the data streams from ROS:
    m_sub_master_target_jaw       = m_nh.subscribe(ros_node_string + "master/target_jaw_position",        1, &PSM::master_target_jawPosCallback, this);
    m_sub_master_target_cart_pos  = m_nh.subscribe(ros_node_string + "master/target_position_cartesian",  1, &PSM::master_target_cartPosCallback, this);
    m_sub_slave_target_jaw        = m_nh.subscribe(ros_node_string + "slave/target_jaw_position",         1, &PSM::slave_target_jawPosCallback, this);
    m_sub_slave_target_cart_pos   = m_nh.subscribe(ros_node_string + "slave/target_position_cartesian",   1, &PSM::slave_target_cartPosCallback, this);

    m_sub_master_joint_angles     = m_nh.subscribe(ros_node_string + "master/joint_angles",               1, &PSM::master_jointAnglesCallback, this);
    m_sub_master_cart_pos         = m_nh.subscribe(ros_node_string + "master/position_cartesian",         1, &PSM::master_cartPosCallback, this);
    m_sub_slave_joint_angles      = m_nh.subscribe(ros_node_string + "slave/joint_angles",                1, &PSM::slave_jointAnglesCallback, this);
    m_sub_slave_cart_pos          = m_nh.subscribe(ros_node_string + "slave/position_cartesian",          1, &PSM::slave_cartPosCallback, this);

    m_sub_robot_state             = m_nh.subscribe(ros_node_string + "slave/robot_state",                 1, &PSM::robot_stateCallback, this);

    //Set up publishers to all of the data streams from ROS:
    m_pub_master_jaw              = m_nh.advertise<cisst_msgs::FloatStamped>  (ros_node_string + "master/set_jaw_position",         1);
    m_pub_master_cart_pos         = m_nh.advertise<geometry_msgs::PoseStamped>(ros_node_string + "master/set_position_cartesian",   1);
    m_pub_slave_jaw               = m_nh.advertise<cisst_msgs::FloatStamped>  (ros_node_string + "slave/set_jaw_position",          1);
    m_pub_slave_cart_pos          = m_nh.advertise<geometry_msgs::PoseStamped>(ros_node_string + "slave/set_position_cartesian",    1);

    m_pub_set_goal_position       = m_nh.advertise<geometry_msgs::PoseStamped>(ros_node_string + "slave/set_position_goal_cartesian",1);
    m_pub_joint_angles            = m_nh.advertise<sensor_msgs::JointState>   (ros_node_string + "slave/set_joint_angles",           1);
    m_pub_joint_angles_goal       = m_nh.advertise<sensor_msgs::JointState>   (ros_node_string + "slave/set_joint_angles_goal",      1);
    m_pub_set_robot_state         = m_nh.advertise<std_msgs::String>          (ros_node_string + "slave/set_robot_state",            1);
}

bool PSM::set_robot_state_and_wait(std::string robotState){
    int iteration = 0;
    ros::Rate loop_rate(1);

    while( ros::ok() && get_robot_state() != robotState){
        set_robot_state(robotState);
        iteration++;

        if(iteration > 10){
          return false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    if(ros::ok() && get_robot_state() == robotState ){
        return true;
    }
    else{
        return false;
    }
}

void PSM::set_robot_state(std::string robotState){
  std_msgs::String msg;
  msg.data = robotState;
  m_pub_set_robot_state.publish(msg);
}

void PSM::set_master_jaw(cisst_msgs::FloatStamped  master_jaw){
  if( m_robot_state != "DVRK_POSITION_CARTESIAN"){
      std::cout << "Command not sent" << std::endl;
      std::cout << "Set state to : DVRK_POSITION_CARTESIAN" << std::endl;
      return;
  }
  m_pub_master_jaw.publish(master_jaw);
}

void PSM::set_master_jaw(float angle){
    cisst_msgs::FloatStamped msg;
    msg.data = angle;
    msg.header.stamp = ros::Time::now();

    set_master_jaw(msg);
}

void PSM::set_master_cart_pos(geometry_msgs::PoseStamped master_cart_pos){
  if( m_robot_state != "DVRK_POSITION_CARTESIAN"){
      std::cout << "Command not sent" << std::endl;
      std::cout << "Set state to : DVRK_POSITION_CARTESIAN" << std::endl;
      return;
  }
  m_pub_master_cart_pos.publish(master_cart_pos);
}

void PSM::set_slave_jaw(cisst_msgs::FloatStamped  slave_jaw){
  if( m_robot_state != "DVRK_POSITION_CARTESIAN" && m_robot_state != "DVRK_POSITION_GOAL_CARTESIAN"){
      std::cout << "Command not sent" << std::endl;
      std::cout << "Set state to : DVRK_POSITION_CARTESIAN or DVRK_POSITION_GOAL_CARTESIAN" << std::endl;
      return;
  }
  m_pub_slave_jaw.publish(slave_jaw);
}

void PSM::set_slave_jaw(float angle){
    cisst_msgs::FloatStamped msg;
    msg.data = angle;
    msg.header.stamp = ros::Time::now();

    set_slave_jaw(msg);
}

void PSM::set_slave_cart_pos(geometry_msgs::PoseStamped slave_cart_pos){
  if( m_robot_state != "DVRK_POSITION_CARTESIAN"){
      std::cout << "Command not sent" << std::endl;
      std::cout << "Set state to : DVRK_POSITION_CARTESIAN" << std::endl;
      return;
  }
  m_pub_slave_cart_pos.publish(slave_cart_pos);
}

void PSM::set_joint_angles(sensor_msgs  ::JointState joint_angles){
  if( m_robot_state != "DVRK_POSITION_JOINT"){
      std::cout << "Command not sent" << std::endl;
      std::cout << "Set state to : DVRK_POSITION_CARTESIAN" << std::endl;
      return;
  }
  m_pub_joint_angles.publish(joint_angles);
}

void PSM::set_pos_goal_cart(geometry_msgs::PoseStamped cart_pos){
  if( m_robot_state != "DVRK_POSITION_GOAL_CARTESIAN"){
      std::cout << "Command not sent" << std::endl;
      std::cout << "Set state to : DVRK_POSITION_GOAL_CARTESIAN" << std::endl;
      return;
  }
  m_pub_set_goal_position.publish(cart_pos);
}

void PSM::set_joint_angles_goal(sensor_msgs::JointState joint_angles){
  if( m_robot_state != "DVRK_POSITION_GOAL_JOINT"){
      std::cout << "Command not sent" << std::endl;
      std::cout << "Set state to : DVRK_POSITION_GOAL_JOINT" << std::endl;
      return;
  }
  m_pub_joint_angles_goal.publish(joint_angles);
}


std::string PSM::get_robot_state(){
  return m_robot_state;
}

cisst_msgs::FloatStamped PSM::get_target_master_jaw(){
  return m_master_target_jaw;
}

geometry_msgs::PoseStamped PSM::get_target_master_cart_pos(){
  return m_master_target_cart_pos;
}

cisst_msgs::FloatStamped PSM::get_target_slave_jaw(){
  return m_slave_target_jaw;
}

geometry_msgs::PoseStamped PSM::get_target_slave_cart_pos(){
  return m_slave_target_cart_pos;
}

sensor_msgs::JointState PSM::get_master_joint_angles(){
  return m_master_joint_angles;
}

geometry_msgs::PoseStamped PSM::get_master_cart_pos(){
  return m_master_cart_pos;
}

cisst_msgs::FloatStamped PSM::get_master_jaw(){
    cisst_msgs::FloatStamped jawMsg;
    jawMsg.header = m_master_joint_angles.header;
    jawMsg.data   = m_master_joint_angles.position[6];

    return jawMsg;
}

sensor_msgs::JointState PSM::get_slave_joint_angles(){
  return m_slave_joint_angles;
}

geometry_msgs::PoseStamped PSM::get_slave_cart_pos(){
  return m_slave_cart_pos;
}

cisst_msgs::FloatStamped PSM::get_slave_jaw(){
    cisst_msgs::FloatStamped jawMsg;
    jawMsg.header = m_slave_joint_angles.header;
    jawMsg.data   = m_slave_joint_angles.position[6];

    return jawMsg;
}


void PSM::master_target_jawPosCallback(const cisst_msgs::FloatStamped& msg){
 m_master_target_jaw = msg;
 emit master_target_jaw_changed(m_master_target_jaw);
}

void PSM::master_target_cartPosCallback(const geometry_msgs::PoseStamped& msg){
  m_master_target_cart_pos = msg;
  emit master_target_cart_pos_changed(m_master_target_cart_pos);
}

void PSM::slave_target_jawPosCallback(const cisst_msgs::FloatStamped& msg){
  m_slave_target_jaw = msg;
  emit slave_target_jaw_changed(m_slave_target_jaw);
}

void PSM::slave_target_cartPosCallback(const geometry_msgs::PoseStamped& msg){
  m_slave_target_cart_pos = msg;
  emit slave_target_cart_pos_changed(m_slave_target_cart_pos);
}

void PSM::master_jointAnglesCallback(const sensor_msgs  ::JointState& msg){
  m_master_joint_angles = msg;
  emit master_joint_angles_changed(m_master_joint_angles);
}

void PSM::master_cartPosCallback(const geometry_msgs::PoseStamped &msg){
  m_master_cart_pos = msg;
  emit master_target_cart_pos_changed(m_master_cart_pos);
}

void PSM::slave_jointAnglesCallback(const sensor_msgs  ::JointState& msg){
  m_slave_joint_angles = msg;
  emit slave_joint_angles_changed(m_slave_joint_angles);
}

void PSM::slave_cartPosCallback(const geometry_msgs::PoseStamped &msg){
  m_slave_cart_pos = msg;
  emit slave_cart_pos_changed(m_slave_cart_pos);
}

void PSM::robot_stateCallback(const std_msgs     ::String& msg){
  m_robot_state = msg.data;
  emit robot_state_changed(m_robot_state);
}
