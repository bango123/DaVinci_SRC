#include <ourStereoVision/include/DeckLinkCaptureDelegate.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <iostream>
#include <string.h>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
