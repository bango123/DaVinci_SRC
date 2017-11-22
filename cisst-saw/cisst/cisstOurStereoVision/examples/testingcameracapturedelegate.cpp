#include <cisstOurStereoVision/include/DeckLinkCaptureDelegate.h>
#include<unistd.h>
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
  //Delay in ms!!
  int delay = 0;

  //Command line inputs for this program
  for(int i = 0; i < argc; i ++){
    if( (0 == strcmp(argv[i], "-d")) && i < argc-1 ){
      delay =  boost::lexical_cast<int>(argv[i+1]);
    }
  }
  std::cout<< delay << std::endl;

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



