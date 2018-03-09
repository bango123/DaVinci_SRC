#include <stereo_vision/DeckLinkCaptureDelegate.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_publisher");

  DeckLinkCaptureDelegate* deckLinkCaptureDelegateL = new DeckLinkCaptureDelegate(ros::NodeHandle("stereo/left") , true);
  DeckLinkCaptureDelegate* deckLinkCaptureDelegateR = new DeckLinkCaptureDelegate(ros::NodeHandle("stereo/right"), false);

  deckLinkCaptureDelegateL->startStream();
  deckLinkCaptureDelegateR->startStream();

  ros::Rate loop_rate(1);
  while (ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
  }

  deckLinkCaptureDelegateL->disconectDeckLink();
  deckLinkCaptureDelegateR->disconectDeckLink();

  ros::shutdown();
  return -1;
}
