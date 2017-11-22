#ifndef DECKLINKCAPTUREDELEGATE_H
#define DECKLINKCAPTUREDELEGATE_H

#include <cisstOurStereoVision/DeckLinkAPI/DeckLinkAPI.h>
#include <cisstOurStereoVision/include/DeckLinkCaptureWorker.h>

#include <opencv2/core/core.hpp>
#include <QMutex>
#include <QObject>
#include <boost/circular_buffer.hpp>
#include <boost/shared_array.hpp>
#include <sensor_msgs/image_encodings.h>



class DeckLinkCaptureDelegate : public IDeckLinkInputCallback
{

public:
  //Delay flag is ms of added delay before image is published
  DeckLinkCaptureDelegate(bool isLeftCamera);

  boost::circular_buffer<cv::Mat> m_FrameQueue            = boost::circular_buffer<cv::Mat>(10);
  boost::circular_buffer<std_msgs::Header> m_HeaderQueue  = boost::circular_buffer<std_msgs::Header>(10);

  bool                      m_isLeftCamera;

  bool                      m_streamRunning           = false;
  clock_t                   m_timeOfLastFrame;
  int                       m_FrameCount              = 0;

  virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
  virtual ULONG STDMETHODCALLTYPE AddRef(void);
  virtual ULONG STDMETHODCALLTYPE  Release(void);

  virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);
  virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*,
                                                            BMDDetectedVideoInputFormatFlags);
  void disconectDeckLink(void);
  void startStream(void);


private:
  int connectToDeckLink(void);
  int setUpCallBack(void);

  DeckLinkCaptureWorker*    workerThread;

  int32_t                   m_refCount;
  //"DeckLink 4K Extreme" for left camera
  //"DeckLink Studio 2" for right camera
  const char *              m_nameOfCard;

  IDeckLink*                m_deckLink                = NULL;
  IDeckLinkInput*           m_deckLinkInput           = NULL;
  IDeckLinkDisplayMode*			m_displayMode             = NULL;

  const char*               m_displayModeName         ="1080i59.94";
  BMDPixelFormat            m_pixelFormat             = bmdFormat8BitYUV;
  BMDVideoInputFlags        m_inputFlags              = bmdVideoInputFlagDefault;

  QMutex mutex;
};

#endif // DECKLINKCAPTUREDELEGATE_H
