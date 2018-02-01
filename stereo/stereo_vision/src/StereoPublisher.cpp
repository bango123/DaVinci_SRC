#include <stereo_vision/DeckLinkCaptureDelegate.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_publisher");

  DeckLinkCaptureDelegate* deckLinkCaptureDelegateL = new DeckLinkCaptureDelegate(true);
  DeckLinkCaptureDelegate* deckLinkCaptureDelegateR = new DeckLinkCaptureDelegate(false);

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
