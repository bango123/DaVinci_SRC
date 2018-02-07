#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>

#include <cisst_msgs/FloatStamped.h>

//buffered image messages for sync:
int sizeOfBuffer_sync = 5;

//"slop" time for synchronization:
float slop = 0.01;

//size of buffer for delay:
int sizeOfBuffer = 35;

//Delay in ms!!
float   delay_s  = 0;
float   delay_ms = 0;

//Message buffers for sync
boost::circular_buffer<sensor_msgs::Image> messageBuffer_L_sync = boost::circular_buffer<sensor_msgs::Image> (sizeOfBuffer_sync);
boost::circular_buffer<sensor_msgs::Image> messageBuffer_R_sync = boost::circular_buffer<sensor_msgs::Image> (sizeOfBuffer_sync);

//Message buffers for delay
boost::circular_buffer<sensor_msgs::Image> messageBuffer_L_delay = boost::circular_buffer<sensor_msgs::Image> (sizeOfBuffer);
boost::circular_buffer<sensor_msgs::Image> messageBuffer_R_delay = boost::circular_buffer<sensor_msgs::Image> (sizeOfBuffer);

void delayCallback(const cisst_msgs::FloatStamped& msg){
    delay_ms = msg.data;
    delay_s = delay_ms/1000.0;
}


void imageCallback_L(const sensor_msgs::ImageConstPtr& msg)
{
  messageBuffer_L_sync.push_back(*msg);
}

void imageCallback_R(const sensor_msgs::ImageConstPtr& msg)
{
  messageBuffer_R_sync.push_back(*msg);
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

  std::cout<< "Camera Sync: Set buffer size to: " << sizeOfBuffer << std::endl;
  std::cout<< "Camera Sync: Slop set to       : " << slop << std::endl;


  //Initialize ros
  ros::init(argc, argv, "sync_delay_publisher");
  ros::NodeHandle nh;

  //Set up to subscribe to the stereo camera feed
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_L = it.subscribe("stereo/left/image_raw"  , 1, imageCallback_L);
  image_transport::Subscriber sub_R = it.subscribe("stereo/right/image_raw" , 1, imageCallback_R);

  //set up publisher for sync:
  image_transport::Publisher pub_L_slave = it.advertise("stereo/slave/left/image_raw" , 1);
  image_transport::Publisher pub_R_slave = it.advertise("stereo/slave/right/image_raw", 1);

  //Set up subscriber for delay value
  ros::Subscriber sub_Delay = nh.subscribe("dvrk/console/teleop/delay"   , 1, delayCallback);

  //set up publisher delay images:
  image_transport::Publisher pub_L_master = it.advertise("stereo/master/left/image_raw" , 1);
  image_transport::Publisher pub_R_master = it.advertise("stereo/master/right/image_raw", 1);


   //Used for FPS caluclation. Just giving initial values!
   ros::Time startTime = ros::Time::now();
   ros::Time minTime;
   float numberOfFramesPublished = 0;

   ros::Rate loop_rate(100);
  while (ros::ok())
  {
    //For the sync/slave side of the system. Will pass sync'ed images to delay buffer
    if(!messageBuffer_L_sync.empty() && !messageBuffer_R_sync.empty()){
       //First case is both are within slop
      if( fabs((messageBuffer_L_sync.front().header.stamp - messageBuffer_R_sync.front().header.stamp).toSec()) < slop ){
           std::cout << "------Frame------" << std::endl;
           std::cout << "Diff between Frame  : " << fabs((messageBuffer_L_sync.front().header.stamp - messageBuffer_R_sync.front().header.stamp).toSec()) << std::endl;

          //Use the minimum timestamp of the two headers!!!
          if(messageBuffer_L_sync.front().header.stamp < messageBuffer_R_sync.front().header.stamp ){
             minTime = messageBuffer_L_sync.front().header.stamp;
             messageBuffer_R_sync.front().header.stamp = messageBuffer_L_sync.front().header.stamp;
          }else{
              minTime = messageBuffer_R_sync.front().header.stamp;
              messageBuffer_L_sync.front().header.stamp = messageBuffer_R_sync.front().header.stamp;
          }
          //Now publish the messages and pop them off the buffer!
          pub_L_slave.publish(messageBuffer_L_sync.front());
          pub_R_slave.publish(messageBuffer_R_sync.front());

          numberOfFramesPublished = numberOfFramesPublished + 1;

          std::cout << "Delay From TimeStamp: " << ros::Time::now() - minTime << std::endl;
          std::cout << "Frame Rate          : " << numberOfFramesPublished/(ros::Time::now() - startTime).toSec() << std::endl;
          std::cout << "Merging L Frame     : " << messageBuffer_L_sync.front().header.seq << std::endl;
          std::cout << "Merging R Frame     : " << messageBuffer_R_sync.front().header.seq << std::endl;

          //Pass sync images to the delay buffer:
          messageBuffer_L_delay.push_back(messageBuffer_L_sync.front());
          messageBuffer_R_delay.push_back(messageBuffer_R_sync.front());

          //Pop the images from the sync buffer
          messageBuffer_L_sync.pop_front();
          messageBuffer_R_sync.pop_front();


          //Reset FPS just in case!
          if(numberOfFramesPublished > 100){
              numberOfFramesPublished = 0;
              startTime = ros::Time::now();
          }
      }
      //If the timestamps are not within slop, then we need to remove the smaller one!
      else{
        if(messageBuffer_L_sync.front().header.stamp < messageBuffer_R_sync.front().header.stamp ){
           messageBuffer_L_sync.pop_front();
        }else{
            messageBuffer_R_sync.pop_front();
        }
      }
    }   // end if for sync buffer

    //Start if for delay buffer:
    if( !messageBuffer_R_delay.empty() && !messageBuffer_L_delay.empty()){

         //The time stamps should be the same for both the left and right buffer. Therefore we only have to check the condition of one!
      if(ros::Time::now() - messageBuffer_R_delay.front().header.stamp > ros::Duration(delay_s)){
         pub_L_master.publish(messageBuffer_L_delay.front());
         pub_R_master.publish(messageBuffer_R_delay.front());

         messageBuffer_L_delay.pop_front();
         messageBuffer_R_delay.pop_front();
      }

    }

    ros::spinOnce();
    loop_rate.sleep();
  } //End of while loop

  return 0;
}
