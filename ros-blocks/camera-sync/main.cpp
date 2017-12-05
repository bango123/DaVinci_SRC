#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>

//buffered image messages:
int sizeOfBuffer = 5;

//"slop" time for synchronization:
float slop = 1.0/30.1;

boost::circular_buffer<sensor_msgs::Image> messageBuffer_L = boost::circular_buffer<sensor_msgs::Image> (sizeOfBuffer);
//boost::circular_buffer<ros::Time>          timeRecieved_L  = boost::circular_buffer<ros::Time>          (sizeOfBuffer);

boost::circular_buffer<sensor_msgs::Image> messageBuffer_R = boost::circular_buffer<sensor_msgs::Image> (sizeOfBuffer);
//boost::circular_buffer<ros::Time>          timeRecieved_R  = boost::circular_buffer<ros::Time>          (sizeOfBuffer);

void imageCallback_L(const sensor_msgs::ImageConstPtr& msg)
{
//  std::cout<< "Left Image recieved" << std::endl;
  messageBuffer_L.push_back(*msg);
//  timeRecieved_L.push_back(ros::Time::now());
}

void imageCallback_R(const sensor_msgs::ImageConstPtr& msg)
{
//  std::cout<< "Left Image recieved" << std::endl;
  messageBuffer_R.push_back(*msg);
//  timeRecieved_R.push_back(ros::Time::now());
}

int main(int argc, char **argv)
{
  //Command line inputs for this program
  for(int i = 0; i < argc; i ++){
    if( (0 == strcmp(argv[i], "-b")) && i < argc-1 ){
      sizeOfBuffer =  boost::lexical_cast<int>(argv[i+1]);
    }
    if( (0 == strcmp(argv[i], "-s")) && i < argc-1 ){
      slop =  boost::lexical_cast<int>(argv[i+1]);
    }
  }
  std::cout<< "Set buffer size to: " << sizeOfBuffer << std::endl;
  std::cout<< "Slop set to       : " << slop << std::endl;


  //Initialize ros
  ros::init(argc, argv, "sync_publisher");
  ros::NodeHandle nh;

  //Set up to subscribe to the stereo camera feed
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_L = it.subscribe("stereo/left/image_raw"  , 1, imageCallback_L);
  image_transport::Subscriber sub_R = it.subscribe("stereo/right/image_raw" , 1, imageCallback_R);

  //set up publisher:
  image_transport::Publisher pub_L = it.advertise("stereo/left_sync/image_raw" , 1);
  image_transport::Publisher pub_R = it.advertise("stereo/right_sync/image_raw", 1);

   //Used for FPS caluclation. Just giving initial values!
   ros::Time startTime = ros::Time::now();
   ros::Time minTime;
   float numberOfFramesPublished = 0;

  while (ros::ok())
  {
    ros::spinOnce();


    if(!messageBuffer_L.empty() && !messageBuffer_R.empty()){
       //First case is both are within slop
      if( abs((messageBuffer_L.front().header.stamp - messageBuffer_R.front().header.stamp).toSec()) < slop ){

          //Use the minimum timestamp of the two headers!!!
          if(messageBuffer_L.front().header.stamp < messageBuffer_R.front().header.stamp ){
             minTime = messageBuffer_L.front().header.stamp;
             messageBuffer_R.front().header.stamp = messageBuffer_L.front().header.stamp;
          }else{
              minTime = messageBuffer_R.front().header.stamp;
              messageBuffer_L.front().header.stamp = messageBuffer_R.front().header.stamp;
          }
          //Now publish the messages and pop them off the buffer!
          pub_L.publish(messageBuffer_L.front());
          pub_R.publish(messageBuffer_R.front());

          numberOfFramesPublished = numberOfFramesPublished + 1;
          std::cout << "------Frame------" << std::endl;
          std::cout << "Delay From TimeStamp: " << ros::Time::now() - minTime << std::endl;
          std::cout << "Frame Rate          : " << numberOfFramesPublished/(ros::Time::now() - startTime).toSec() << std::endl;
          std::cout << "Merging L Frame     : " << messageBuffer_L.front().header.seq << std::endl;
          std::cout << "Merging R Frame     : " << messageBuffer_R.front().header.seq << std::endl;



          messageBuffer_L.pop_front();
          messageBuffer_R.pop_front();



          //Reset FPS just in case!
          if(numberOfFramesPublished > 100){
              numberOfFramesPublished = 0;
              startTime = ros::Time::now();
          }
      }
      //If the timestamps are not within slop, then we need to remove the smaller one!
      else{
        if(messageBuffer_L.front().header.stamp < messageBuffer_R.front().header.stamp ){
           messageBuffer_L.pop_front();
        }else{
            messageBuffer_R.pop_front();
        }
      }

    }


  }

  return 0;
}
