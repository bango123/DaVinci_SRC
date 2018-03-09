#ifndef DECKLINKCAPTUREDELEGATE_H
#define DECKLINKCAPTUREDELEGATE_H

#include "../../DeckLinkAPI/DeckLinkAPI.h"

#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>


class DeckLinkCaptureDelegate : public IDeckLinkInputCallback
{

public:
  //Delay flag is ms of added delay before image is published
  DeckLinkCaptureDelegate(ros::NodeHandle nh, bool isLeftCamera);

  virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
  virtual ULONG   STDMETHODCALLTYPE AddRef(void);
  virtual ULONG   STDMETHODCALLTYPE Release(void);

  virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);
  virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*,
                                                            BMDDetectedVideoInputFormatFlags);
  void disconectDeckLink(void);
  void startStream(void);


private:
  int connectToDeckLink(void);
  int setUpCallBack(void);

  bool m_isLeftCamera;

  bool m_streamRunning = false;

  int32_t                   m_refCount;
  //"DeckLink 4K Extreme" for left camera
  //"DeckLink Studio 2" for right camera
  const char *              m_nameOfCard;

  IDeckLink*                m_deckLink                = NULL;
  IDeckLinkInput*           m_deckLinkInput           = NULL;
  IDeckLinkDisplayMode*		m_displayMode             = NULL;

  const char*               m_displayModeName         ="1080i59.94";
  BMDPixelFormat            m_pixelFormat             = bmdFormat8BitYUV;
  BMDVideoInputFlags        m_inputFlags              = bmdVideoInputFlagDefault;

  //Ros stuff
  ros::NodeHandle                         m_nh;
  image_transport::ImageTransport         m_it;
  image_transport::CameraPublisher        m_image_pub;
  camera_info_manager::CameraInfoManager  m_cinfo;

};

#endif // DECKLINKCAPTUREDELEGATE_H
