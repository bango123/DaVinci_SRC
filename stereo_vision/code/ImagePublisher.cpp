#include <stereo_vision/DeckLinkCaptureDelegate.h>
#include <ros/ros.h>


//command line input is r or l for right or left camera publisher
int main(int argc, char** argv)
{
    bool m_isLeft = true;
    if(argv[1][0] == 'r'){
        m_isLeft = false;
    }

    DeckLinkCaptureDelegate* deckLinkCaptureDelegate;

    if(m_isLeft){
        ros::init(argc, argv, "left_camera_publisher");
        deckLinkCaptureDelegate = new DeckLinkCaptureDelegate(ros::NodeHandle("stereo/left") , true);
    }
    else{
        ros::init(argc, argv, "right_camera_publisher");
        deckLinkCaptureDelegate = new DeckLinkCaptureDelegate(ros::NodeHandle("stereo/right"), false);
    }


  deckLinkCaptureDelegate->startStream();

  ros::Rate loop_rate(1);
  while (ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
  }

  deckLinkCaptureDelegate->disconectDeckLink();

  ros::shutdown();
  return -1;
}
