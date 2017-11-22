#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>

//Delay in ms!!
float delay = 0;

//buffered image messages:
int sizeOfBuffer = 20;
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
  //Command line inputs for this program
  for(int i = 0; i < argc; i ++){
    if( (0 == strcmp(argv[i], "-d")) && i < argc-1 ){
      delay =  boost::lexical_cast<float>(argv[i+1]);
    }
    if( (0 == strcmp(argv[i], "-b")) && i < argc-1 ){
      sizeOfBuffer =  boost::lexical_cast<int>(argv[i+1]);
    }
  }
  std::cout<< "Delay set to (mS) : " << delay       << std::endl;
  delay = delay/1000.0;
  std::cout<< "Set buffer size to: " << sizeOfBuffer << std::endl;


  //Initialize ros
  ros::init(argc, argv, "delay_publisher");
  ros::NodeHandle nh;

  //Set up to subscribe to the stereo camera feed
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_L = it.subscribe("stereo/left/image_raw"  , 1, imageCallback_L);
  image_transport::Subscriber sub_R = it.subscribe("stereo/right/image_raw" , 1, imageCallback_R);

  //set up publisher:
  image_transport::Publisher pub_L = it.advertise("stereo/left_delay/image_raw" , 1);
  image_transport::Publisher pub_R = it.advertise("stereo/right_delay/image_raw", 1);

  //image_transport::Subscriber subR_image = it.subscribe("stereo/right/image_raw", 1, imageCallback_R);

  //ros::spin();
  while (ros::ok())
  {
    ros::spinOnce();

    if(!messageBuffer_L.empty()){
      if(ros::Time::now() - timeRecieved_L.front() > ros::Duration(delay)){
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
      if(ros::Time::now() - timeRecieved_R.front() > ros::Duration(delay)){
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

  return 0;
}
