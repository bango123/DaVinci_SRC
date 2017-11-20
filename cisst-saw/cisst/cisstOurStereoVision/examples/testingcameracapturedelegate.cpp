#include <cisstOurStereoVision/include/DeckLinkCaptureDelegate.h>
#include<unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");

  DeckLinkCaptureDelegate* deckLinkCaptureDelegateL = new DeckLinkCaptureDelegate(true);
  DeckLinkCaptureDelegate* deckLinkCaptureDelegateR = new DeckLinkCaptureDelegate(false);

  deckLinkCaptureDelegateL->startStream();
  deckLinkCaptureDelegateR->startStream();


  while (true){}

  deckLinkCaptureDelegateL->disconectDeckLink();
  deckLinkCaptureDelegateR->disconectDeckLink();
  return -1;
}



