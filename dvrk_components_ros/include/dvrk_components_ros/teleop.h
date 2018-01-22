#ifndef TELEOP_H
#define TELEOP_H

#include <QObject>
#include <ros/ros.h>

#include <cisst_msgs/FloatStamped.h>
#include <cisst_msgs/BoolStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

class Teleop : public QObject
{
  Q_OBJECT
public:
  explicit Teleop(ros::NodeHandle nh, QObject *parent = nullptr);

  //high level turn on commands
  bool reset_dvrk();
  bool turn_onTeleop();
  bool set_ros_only_wait(bool rosOnly);

  //Setters
  void set_delay       (float delay);
  void set_scale       (float scale);

  void set_ros_only    (bool rosOnly);
  void set_powerStatus (bool powerStatus);
  void set_teleopStatus(bool teleopStatus);

  //Getters
  float get_delay();
  float get_scale();

  bool get_clutch_pressed();
  bool get_camera_pressed();
  bool get_coag_pressed();
  bool get_cameraPlus_pressed();
  bool get_cameraMinus_pressed();

  bool get_powerStatus();
  bool get_rosOnly();
  bool get_teleopStatus();
  bool get_operatorPresent();

signals:
  void delay_changed      (float delay);
  void scale_changed      (float scale);

  void clutch_changed     (bool pressed);
  void camera_changed     (bool pressed);
  void coag_changed       (bool pressed);
  void cameraPlus_changed (bool pressed);
  void cameraMinus_changed(bool pressed);

  //These signals will be sent for both gui clicks and ros submitted changes
  void powerStatus_changed (bool power_status);
  void rosOnly_changed     (bool ros_only);
  void teleopStatus_changed(bool teleop_status);

  void operatorPresent_changed(bool operatorPresent);

private:
  ros::NodeHandle m_nh;

  //In mS
  float m_delay;
  float m_scale;

  //Footpedals
  bool m_clutch_pressed;
  bool m_camera_pressed;
  bool m_coag_pressed;
  bool m_cameraPlus_pressed;
  bool m_cameraMinus_pressed;

  //console states
  bool m_home;
  bool m_ros_only;
  bool m_teleop_running;

  bool m_operatorPresent;

  //Subscribers
  ros::Subscriber m_sub_delay;
  ros::Subscriber m_sub_scale;

  ros::Subscriber m_sub_clutch;
  ros::Subscriber m_sub_camera;
  ros::Subscriber m_sub_coag;
  ros::Subscriber m_sub_cameraPlus;
  ros::Subscriber m_sub_cameraMinus;

  ros::Subscriber m_sub_ros_only;
  ros::Subscriber m_sub_teleop_status;
  ros::Subscriber m_sub_power_status;

  //Callbacks for subscribers
  void delayCallback(const cisst_msgs::FloatStamped& msg);
  void scaleCallback(const cisst_msgs::FloatStamped& msg);

  void clutchCallback     (const sensor_msgs::Joy& msg);
  void cameraCallback     (const sensor_msgs::Joy& msg);
  void coagCallback       (const sensor_msgs::Joy& msg);
  void cameraPlusCallback (const sensor_msgs::Joy& msg);
  void cameraMinusCallback(const sensor_msgs::Joy& msg);

  void rosOnlyCallback     (const cisst_msgs::BoolStamped& msg);
  void teleopStatusCallback(const cisst_msgs::BoolStamped& msg);
  void powerStatusCallback (const cisst_msgs::BoolStamped& msg);


  //Publishers
  ros::Publisher m_pub_delay;
  ros::Publisher m_pub_scale;

  ros::Publisher m_pub_ros_only;
  ros::Publisher m_pub_home;
  ros::Publisher m_pub_power_off;
  ros::Publisher m_pub_teleop_enable;


};

#endif // TELEOP_H
