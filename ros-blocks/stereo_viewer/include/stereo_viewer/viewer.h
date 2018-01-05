#ifndef VIEWER_H
#define VIEWER_H


#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<image_transport/camera_subscriber.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <string>

class Viewer 
{

public: 
	//pos_x and pos_y are the pixel value coordinates of the display to fullscreen on	
  Viewer(ros::NodeHandle nh, const std::string& image_topic1, int pos_x1, int pos_y1, const std::string& image_topic2, int pos_x2, int pos_y2);
	~Viewer();

	void run();

private:
	//keep track of the settings:
	bool m_isFullScreen;

  std::string 				m_image_topic1;
  std::string 				m_image_topic2;
  ros::NodeHandle			m_nh;
	image_transport::ImageTransport 	m_it;

  image_transport::Subscriber		m_sub1;
  image_transport::Subscriber		m_sub2;

	bool					m_running = false;

  //Keeps track of the incomming images
  bool new_img;
  cv::Mat m_img1;
  cv::Mat m_img2;

  cv_bridge::CvImageConstPtr cv_ptr1;
  cv_bridge::CvImageConstPtr cv_ptr2;



  ros::Time m_img1_ts;
  ros::Time m_img2_ts;
	
  void display_callback1(const sensor_msgs::ImageConstPtr& msg);
  void display_callback2(const sensor_msgs::ImageConstPtr& msg);

	void loop();


};


#endif // VIEWER_H
