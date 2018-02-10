#include <dvrk_components_ros/teleop.h>
#include <stdio.h>

Teleop::Teleop(ros::NodeHandle nh, QObject *parent) : QObject(parent),
  m_nh(nh),
  m_clutch_pressed(false),
  m_camera_pressed(false),
  m_coag_pressed(false),
  m_cameraPlus_pressed(false),
  m_cameraMinus_pressed(false),
  m_home(false),
  m_teleop_running(false),
  m_operatorPresent(false)
{
  //Set up all of the subscribers
  m_sub_delay         = m_nh.subscribe("dvrk/console/teleop/delay",    1, &Teleop::delayCallback,      this);
  m_sub_scale         = m_nh.subscribe("dvrk/console/teleop/scale",    1, &Teleop::scaleCallback,      this);

  m_sub_clutch        = m_nh.subscribe("dvrk/footpedals/clutch",       1, &Teleop::clutchCallback,      this);
  m_sub_camera        = m_nh.subscribe("dvrk/footpedals/camera",       1, &Teleop::cameraCallback,      this);
  m_sub_coag          = m_nh.subscribe("dvrk/footpedals/coag",         1, &Teleop::coagCallback  ,      this);
  m_sub_cameraPlus    = m_nh.subscribe("dvrk/footpedals/camera_plus",  1, &Teleop::cameraPlusCallback,  this);
  m_sub_cameraMinus   = m_nh.subscribe("dvrk/footpedals/camera_minus", 1, &Teleop::cameraMinusCallback, this);

  m_sub_ros_only      = m_nh.subscribe("dvrk/console/teleop/ros_only", 1, &Teleop::rosOnlyCallback,     this);
  m_sub_teleop_status = m_nh.subscribe("dvrk/console/teleop/status",   1, &Teleop::teleopStatusCallback,this);
  m_sub_power_status  = m_nh.subscribe("dvrk/console/power_status" ,   1, &Teleop::powerStatusCallback, this);

  //Set up all of the publishers
  m_pub_delay         = m_nh.advertise<cisst_msgs::FloatStamped>("dvrk/console/teleop/set_delay",  1);
  m_pub_scale         = m_nh.advertise<cisst_msgs::FloatStamped>("dvrk/console/teleop/set_scale",  1);

  m_pub_ros_only      = m_nh.advertise<cisst_msgs::BoolStamped>("dvrk/console/teleop/set_ros_only",1);
  m_pub_home          = m_nh.advertise<std_msgs  ::Empty      >("dvrk/console/set_home",           1);
  m_pub_power_off     = m_nh.advertise<std_msgs  ::Empty      >("dvrk/console/set_power_off",      1);
  m_pub_teleop_enable = m_nh.advertise<cisst_msgs::BoolStamped>("dvrk/console/teleop/set_enable",  1);

}

bool Teleop::reset_dvrk(){
  ros::Rate loop_rate(1);
  int iteration = 0;

  //turn off dvrk first!
  //Just to wait for a second so it fully turns off
  while(ros::ok()){
    set_powerStatus(false);

    iteration++;
    if(iteration > 10){
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  if(get_powerStatus()){
      return false;
  }

  iteration = 0;

  //Continue looping turn_on to DVRK till it is on
  //Loop continues every 1 second and only tries 10 times
  while(!get_powerStatus() && ros::ok()){
    set_powerStatus(true);

    iteration++;
    if(iteration > 10){
      return false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  if(ros::ok() && get_powerStatus()){
    return true;
  }
  else{
    return false;
  }}

bool Teleop::turn_onTeleop(){
  int iteration = 0;
  ros::Rate loop_rate(0.5);

  //Continue looping set_teleopStatus till we have feedback it is on!
  //Loop continues every 2 seconds and only tries 5 times
  while(!get_teleopStatus() && ros::ok()){
    set_teleopStatus(true);

    iteration++;
    if(iteration > 10){
      return false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  if(ros::ok() && get_teleopStatus()){
    return true;
  }
  else{
    return false;
  }
}

bool Teleop::set_ros_only_wait(bool rosOnly){
  set_ros_only(rosOnly);

  int iteration = 0;
  ros::Rate loop_rate(1);

  //Continue looping till the set_ros_only is equal to our target
  //Loops 10 times and once every second.
  while(rosOnly != get_rosOnly() && ros::ok()){
    iteration++;
    if(iteration > 10){
      return false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  if(ros::ok() && rosOnly == get_rosOnly()){
    return true;
  }
  else{
    return false;
  }
}

void Teleop::set_delay(float delay){
  cisst_msgs::FloatStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.data = delay;
  m_pub_delay.publish(msg);
}

void Teleop::set_scale(float scale){
  cisst_msgs::FloatStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.data = scale;
  m_pub_scale.publish(msg);
}

void Teleop::set_ros_only(bool rosOnly){
   cisst_msgs::BoolStamped msg;
   msg.header.stamp = ros::Time::now();
   msg.data = rosOnly;
   m_pub_ros_only.publish(msg);
}

void Teleop::set_powerStatus(bool powerStatus){
   std_msgs::Empty msg;

   if(powerStatus){
      m_pub_home.publish(msg);
   }
   else{
      m_pub_power_off.publish(msg);
   }
}

void Teleop::set_teleopStatus(bool teleopStatus){
   cisst_msgs::BoolStamped msg;
   msg.header.stamp = ros::Time::now();
   msg.data = teleopStatus;
   m_pub_teleop_enable.publish(msg);
}

float Teleop::get_delay(){
  return m_delay;
}

float Teleop::get_scale(){
  return m_scale;
}

bool Teleop::get_clutch_pressed(){
  return m_clutch_pressed;
}

bool Teleop::get_camera_pressed(){
  return m_camera_pressed;
}

bool Teleop::get_coag_pressed(){
  return m_coag_pressed;
}

bool Teleop::get_cameraPlus_pressed(){
  return m_cameraPlus_pressed;
}

bool Teleop::get_cameraMinus_pressed(){
  return m_cameraMinus_pressed;
}

bool Teleop::get_powerStatus(){
  return m_home;
}

bool Teleop::get_rosOnly(){
  return m_ros_only;
}

bool Teleop::get_teleopStatus(){
  return m_teleop_running;
}

bool Teleop::get_operatorPresent(){
  return m_operatorPresent;
}

void Teleop::delayCallback(const cisst_msgs::FloatStamped& msg){
  m_delay = msg.data;
  emit delay_changed(m_delay);
}

void Teleop::scaleCallback(const cisst_msgs::FloatStamped& msg){
  m_scale = msg.data;
  emit scale_changed(m_scale);
}

void Teleop::clutchCallback(const sensor_msgs::Joy& msg ){
  m_clutch_pressed = msg.buttons[0];
  emit clutch_changed(m_clutch_pressed);
}

void Teleop::cameraCallback(const sensor_msgs::Joy& msg ){
  m_camera_pressed = msg.buttons[0];
  emit camera_changed(m_camera_pressed);
}

void Teleop::coagCallback(const sensor_msgs::Joy& msg ){
  m_coag_pressed = msg.buttons[0];
  emit coag_changed(m_coag_pressed);

  //toggle the state of if the operator is present on press
  if(m_coag_pressed){
    m_operatorPresent = !m_operatorPresent;
    emit operatorPresent_changed(m_operatorPresent);
  }
}

void Teleop::cameraPlusCallback(const sensor_msgs::Joy& msg ){
  m_cameraPlus_pressed = msg.buttons[0];
  emit cameraPlus_changed(m_cameraPlus_pressed);
}

void Teleop::cameraMinusCallback(const sensor_msgs::Joy& msg ){
  m_cameraMinus_pressed = msg.buttons[0];
  emit cameraMinus_changed(m_cameraMinus_pressed);
}

void Teleop::rosOnlyCallback(const cisst_msgs::BoolStamped& msg){
  m_ros_only = msg.data;
  emit rosOnly_changed(m_ros_only);
}

void Teleop::teleopStatusCallback(const cisst_msgs::BoolStamped& msg){
  m_teleop_running = msg.data;
  emit teleopStatus_changed(m_teleop_running);
}

void Teleop::powerStatusCallback(const cisst_msgs::BoolStamped& msg){
  m_home = msg.data;
  emit powerStatus_changed(m_home);
}
