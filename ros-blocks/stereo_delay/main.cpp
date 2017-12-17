#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>

//Delay in ms!!
float delay_s  = 0;
int   delay_ms = 0;

//buffered image messages:
int sizeOfBuffer = 35;
boost::circular_buffer<sensor_msgs::Image> messageBuffer_L = boost::circular_buffer<sensor_msgs::Image> (sizeOfBuffer);
boost::circular_buffer<ros::Time>          timeRecieved_L  = boost::circular_buffer<ros::Time>          (sizeOfBuffer);

boost::circular_buffer<sensor_msgs::Image> messageBuffer_R = boost::circular_buffer<sensor_msgs::Image> (sizeOfBuffer);
boost::circular_buffer<ros::Time>          timeRecieved_R  = boost::circular_buffer<ros::Time>          (sizeOfBuffer);

void imageCallback_L(const sensor_msgs::ImageConstPtr& msg)
{
//  std::cout<< "Left Image recieved" << std::endl;
  messageBuffer_L.push_back(*msg);
  timeRecieved_L.push_back(ros::Time::now());
}

void imageCallback_R(const sensor_msgs::ImageConstPtr& msg)
{
//  std::cout<< "Left Image recieved" << std::endl;
  messageBuffer_R.push_back(*msg);
  timeRecieved_R.push_back(ros::Time::now());
}

int main(int argc, char **argv)
{

  //Initialize ros
  ros::init(argc, argv, "delay_publisher");
  ros::NodeHandle nh;

  //Iniatlize delay
  if(!nh.hasParam("delay")){
    nh.setParam("delay", delay_ms);
  }
  else{
    nh.getParam("delay", delay_ms);
  }
  delay_s = delay_ms/1000.0;

  std::cout<< "Camera Delay: Delay set to (mS) : " << delay_ms     << std::endl;
  std::cout<< "Set buffer size to: " << sizeOfBuffer << std::endl;

  //Set up to subscribe to the stereo camera feed
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_L = it.subscribe("stereo/left_sync/image_raw"  , 1, imageCallback_L);
  image_transport::Subscriber sub_R = it.subscribe("stereo/right_sync/image_raw" , 1, imageCallback_R);

  //set up publisher:
  image_transport::Publisher pub_L = it.advertise("stereo/left_delay/image_raw" , 1);
  image_transport::Publisher pub_R = it.advertise("stereo/right_delay/image_raw", 1);

  //image_transport::Subscriber subR_image = it.subscribe("stereo/right/image_raw", 1, imageCallback_R);

  //ros::spin();
  while (ros::ok())
  {
    ros::spinOnce();

    nh.getParam("delay", delay_ms);
    delay_s = delay_ms/1000.0;

    if(!messageBuffer_L.empty()){
      if(ros::Time::now() - timeRecieved_L.front() > ros::Duration(delay_s)){
        pub_L.publish(messageBuffer_L.front());
        std::cout << "----Left Frame ----" << std::endl;
        std::cout << "Frame ID                  : " << messageBuffer_L.front().header.seq                      << std::endl;
        std::cout << "Time Stamp                : " << messageBuffer_L.front().header.stamp                    << std::endl;
        std::cout << "Added Delay to Time Stamp : " << ros::Time::now() - messageBuffer_L.front().header.stamp << std::endl;
        std::cout << "Added Delay               : " << ros::Time::now() - timeRecieved_L.front()               << std::endl;

        messageBuffer_L.pop_front();
        timeRecieved_L.pop_front();
      }
    }

    if(!messageBuffer_R.empty()){
      if(ros::Time::now() - timeRecieved_R.front() > ros::Duration(delay_s)){
        pub_R.publish(messageBuffer_R.front());
        std::cout << "----Right Frame----" << std::endl;
        std::cout << "Frame ID                  : " << messageBuffer_R.front().header.seq                      << std::endl;
        std::cout << "Time Stamp                : " << messageBuffer_R.front().header.stamp                    << std::endl;
        std::cout << "Added Delay to Time Stamp : " << ros::Time::now() - messageBuffer_R.front().header.stamp << std::endl;
        std::cout << "Added Delay               : " << ros::Time::now() - timeRecieved_R.front()               << std::endl;

        messageBuffer_R.pop_front();
        timeRecieved_R.pop_front();
      }
    }


  }

  //Clean up the parameters
  nh.deleteParam("delay");

  return 0;
}
