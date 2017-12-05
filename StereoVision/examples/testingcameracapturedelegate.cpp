#include <StereoVision/DeckLinkCaptureDelegate.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_publisher");

  DeckLinkCaptureDelegate* deckLinkCaptureDelegateL = new DeckLinkCaptureDelegate(true);
  DeckLinkCaptureDelegate* deckLinkCaptureDelegateR = new DeckLinkCaptureDelegate(false);

  deckLinkCaptureDelegateL->startStream();
  deckLinkCaptureDelegateR->startStream();


  while (ros::ok()){}

  deckLinkCaptureDelegateL->disconectDeckLink();
  deckLinkCaptureDelegateR->disconectDeckLink();

  ros::shutdown();
  return -1;
}
