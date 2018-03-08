#include <stereo_vision/DeckLinkCaptureDelegate.h>

#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>



DeckLinkCaptureDelegate::DeckLinkCaptureDelegate(ros::NodeHandle nh, bool isLeftCamera):
    m_refCount(1),
    m_isLeftCamera(isLeftCamera),
    m_nh(nh),
    m_cinfo(m_nh),
    m_it(m_nh),
    m_image_pub( m_it.advertiseCamera("image_raw", 1) )
{
  if (m_isLeftCamera){
    m_nameOfCard = "DeckLink 4K Extreme";
  }
  else{
    m_nameOfCard = "DeckLink Studio 2";
  }

  int                 error;

   //Connect to the DeckLink
   error =  connectToDeckLink();
   if(error == -1){
     return;
    }

   //Set up the callback!
   error = setUpCallBack();
   if(error == -1){
     return;
    }

    //Set up rosnode stuff
   if(m_isLeftCamera){
     m_cinfo.setCameraName("left");
   }
   else{
     m_cinfo.setCameraName("right");
   }

   m_cinfo.loadCameraInfo("file://${ROS_HOME}/camera_info/${NAME}.yaml");

   return;
}

ULONG DeckLinkCaptureDelegate::AddRef(void)
{
  return __sync_add_and_fetch(&m_refCount, 1);
}

ULONG DeckLinkCaptureDelegate::Release(void)
{
  int32_t newRefValue = __sync_sub_and_fetch(&m_refCount, 1);
  if (newRefValue == 0)
  {
    delete this;
    return 0;
  }
  return newRefValue;
}

HRESULT DeckLinkCaptureDelegate::VideoInputFrameArrived(IDeckLinkVideoInputFrame* videoFrame, IDeckLinkAudioInputPacket* audioFrame)
{
  if(!ros::ok()){
    disconectDeckLink();
  }



  if( !videoFrame){
    printf("Video Frame is Null\n");
    return E_FAIL;
  }

  //Convert decklinkInput Frame to Mat from OpenCV
  void*	frameBytes;
  videoFrame->GetBytes(&frameBytes);
  cv::Mat frame(videoFrame->GetHeight(), videoFrame->GetWidth(), CV_8UC2, frameBytes, videoFrame->GetRowBytes());

  cv::cvtColor(frame, frame, CV_YUV2RGB_UYVY);

  //Convert Mat from OpenCV to ROS msg
  cv_bridge::CvImage out_msg;
  std_msgs::Header header = std_msgs::Header();
  header.stamp = ros::Time::now();

  out_msg.header   = header;
  out_msg.encoding = sensor_msgs::image_encodings::RGB8;
  out_msg.image    = frame;

  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(m_cinfo.getCameraInfo()));

  ci->header.frame_id = out_msg.header.frame_id;
  ci->header.stamp    = out_msg.header.stamp;

  ci->height = frame.rows;
  ci->width  = frame.cols;

  m_image_pub.publish(out_msg.toImageMsg(), ci);
  frame.release();

  return S_OK;
}

HRESULT DeckLinkCaptureDelegate::VideoInputFormatChanged(BMDVideoInputFormatChangedEvents events, IDeckLinkDisplayMode *mode, BMDDetectedVideoInputFormatFlags formatFlags)
{
  printf("Video Input Format Changed\n");

  return S_OK;
}

void DeckLinkCaptureDelegate::startStream(void){
  m_deckLinkInput->StartStreams();
  printf("Starting Stream for %s\n", m_nameOfCard);
  m_streamRunning = true;
}

//This will connect to the m_DeckLink and m_deckLinkInput
//returns -1 if an error occurs :(
int DeckLinkCaptureDelegate::connectToDeckLink(void){
    IDeckLinkIterator*	deckLinkIterator;
    bool                foundCard = false;
    HRESULT             result;

    // Create an IDeckLinkIterator object to enumerate all DeckLink cards in the system
    deckLinkIterator = CreateDeckLinkIteratorInstance();
    if (deckLinkIterator == NULL)
    {
      fprintf(stderr, "A DeckLink iterator could not be created.  The DeckLink drivers may not be installed.\n");
      return -1;
    }

    // Go thorugh all the cards and find the card equal to nameOfCard
    while (deckLinkIterator->Next(&m_deckLink) == S_OK)
    {
      char *		deviceNameString = NULL;

      result = m_deckLink->GetModelName((const char **) &deviceNameString);
      if (result == S_OK)
      {
        if (strcmp(m_nameOfCard, deviceNameString) == 0)
        {
          printf("Found: %s\n", deviceNameString);
          foundCard = true;
          free(deviceNameString);
          break;
        }
      }
      free(deviceNameString);
    }

    //Release the deckLinkIterator since we went through all the cards
    deckLinkIterator->Release();

    // if we did not find the card, then we should error and return
    if (!foundCard){
      m_deckLink->Release();
      fprintf(stderr, "Decklink card %s could not be found.\n", m_nameOfCard);
      return -1;
    }
    result = m_deckLink->QueryInterface(IID_IDeckLinkInput, (void**)&m_deckLinkInput);
    if (result != S_OK)
    {
      fprintf(stderr, "DeckLinkInput could not be found.\n");
      m_deckLink->Release();
      return -1;
    }

    return 1;
}

//returns -1 if an error occurs :(
int DeckLinkCaptureDelegate::setUpCallBack(void){
  HRESULT             result;

  m_deckLinkInput->SetCallback(this);

  IDeckLinkDisplayModeIterator*	displayModeIterator = NULL;

  result = m_deckLinkInput->GetDisplayModeIterator(&displayModeIterator);
    if (result != S_OK){
      fprintf(stderr, "Failed to create Display Mode Iterator");
      disconectDeckLink();
      return -1;
    }
    while ((result = displayModeIterator->Next(&m_displayMode)) == S_OK)
    {
      char*	displayModeName;
      m_displayMode->GetName((const char**)&displayModeName);

      if (strcmp(m_displayModeName, displayModeName) == 0)
        break;

      m_displayMode->Release();
      free(displayModeName);
    }

    if (result != S_OK || m_displayMode == NULL)
    {
      fprintf(stderr, "Unable to get display mode %s\n", m_displayModeName);
      disconectDeckLink();
      return -1;
    }

  result = m_deckLinkInput->EnableVideoInput(m_displayMode->GetDisplayMode(), m_pixelFormat, m_inputFlags);
  if (result != S_OK)
  {
    fprintf(stderr, "Failed to enable video input. Is another application using the card?\n");
    disconectDeckLink();
    return -1;
  }

  return 1;
}

//Disconnects all the stuff!!!
void DeckLinkCaptureDelegate::disconectDeckLink(void){
    printf("Disconnecting %s\n", m_nameOfCard);

    if( m_streamRunning ){
        m_deckLinkInput->StopStreams();
        m_deckLinkInput->FlushStreams();
        m_streamRunning = false;
    }
    if (m_displayMode != NULL){
      m_displayMode->Release();
    }
    if (m_deckLinkInput != NULL){
      m_deckLinkInput->Release();
    }
    if (m_deckLink != NULL){
      m_deckLink->Release();
    }

    this->Release();

    return;
}
