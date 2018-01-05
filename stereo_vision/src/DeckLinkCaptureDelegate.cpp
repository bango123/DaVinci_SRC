#include <stereo_vision/DeckLinkCaptureDelegate.h>

#include <iostream>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>



DeckLinkCaptureDelegate::DeckLinkCaptureDelegate(bool isLeftCamera):
    m_refCount(1),
    m_isLeftCamera(isLeftCamera)

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

   //Create worker class
   if(m_isLeftCamera){
      ros::NodeHandle nh("stereo/left");
      workerThread = new PublishImageWorker(m_isLeftCamera, nh);
   }
   else{
      ros::NodeHandle nh("stereo/right");
      workerThread = new PublishImageWorker(m_isLeftCamera, nh);
   }

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

//This code gets ran when a frame arrives!!!!
HRESULT DeckLinkCaptureDelegate::VideoInputFrameArrived(IDeckLinkVideoInputFrame* videoFrame, IDeckLinkAudioInputPacket* audioFrame)
{
  if(!ros::ok()){
    disconectDeckLink();
  }

  m_FrameCount++;

  //First timestamp!!!
  m_HeaderQueue.push_back(std_msgs::Header());
  m_HeaderQueue.front().stamp = ros::Time::now();
  //m_HeaderQueue.front().seq = m_FrameCount;
  //m_HeaderQueue.front().frame_id = "0";

  if (m_isLeftCamera){
    printf("--------- Left New Frame---------\n");
  }
  else{
    printf("---------Right New Frame---------\n");
  }

  //clock_t currentTime = clock();
  std::cout << "Frame Rate: " << ( 1 )/(ros::Time::now() - m_timeOfLastFrame).toSec() << std::endl;

  m_timeOfLastFrame = ros::Time::now();

  std::cout << "Time Stamp: " << m_HeaderQueue.front().stamp << std::endl;
  std::cout << "Sequence  : " << m_HeaderQueue.front().seq << std::endl;
  std::cout << "Frame ID  : " << m_HeaderQueue.front().frame_id << std::endl;

  if( !videoFrame){
    printf("Video Frame is Null\n");
    return E_FAIL;
  }

  void*	frameBytes;
  videoFrame->GetBytes(&frameBytes);

  mutex.lock();
  m_FrameQueue.push_back(cv::Mat(videoFrame->GetHeight(), videoFrame->GetWidth(), CV_8UC2, frameBytes, videoFrame->GetRowBytes()));
  mutex.unlock();

  std::cout << "Size of buffer: " << m_FrameQueue.size() << std::endl;

  if( !workerThread->isRunning() && ros::ok()){
    workerThread->setFrame( m_FrameQueue.front(), m_HeaderQueue.front());
    workerThread->start();

    mutex.lock();
    m_FrameQueue.pop_front();
    mutex.unlock();
    m_HeaderQueue.pop_front();
  }

  if( m_FrameCount%30 == 0){
    m_FrameQueue.erase( m_FrameQueue.begin(),  m_FrameQueue.end());
    m_HeaderQueue.erase(m_HeaderQueue.begin(), m_HeaderQueue.end());
  }

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
