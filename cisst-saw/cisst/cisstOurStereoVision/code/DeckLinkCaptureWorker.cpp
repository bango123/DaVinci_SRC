#include <cisstOurStereoVision/include/DeckLinkCaptureWorker.h>

#include <opencv2/core/core.hpp>
//#include <opencv2/core/cuda.hpp>
//#include <opencv2/cudaimgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/time.h>

#include <QMutex>

#include <stdio.h>
#include <stdlib.h>

#include <signal.h>
#include <execinfo.h>

void handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}


DeckLinkCaptureWorker::DeckLinkCaptureWorker( bool isLeftCamera) :
    m_isLeftCamera(isLeftCamera),
    it_(nh_)
{
  signal(SIGSEGV, handler);
    if( m_isLeftCamera){
      image_pub_ = it_.advertise("stereo/left", 1);
    }
    else{
      image_pub_ = it_.advertise("stereo/right", 1);
    }
}


void DeckLinkCaptureWorker::setFrame(const cv::Mat frame, const std_msgs::Header header){
  //mutex.lock();
  m_frame = frame.clone();
  m_header = header;
  //mutex.unlock();
}

void DeckLinkCaptureWorker::run() {

  cv_bridge::CvImage out_msg;
  out_msg.header   = m_header;
  out_msg.encoding = sensor_msgs::image_encodings::RGB8;

  cv::Rect roi;
  roi.x = 160;
  roi.y = 15;
  roi.width = 1400;
  roi.height = 1050;

  m_frame = m_frame(roi);

  //mutex.lock();
  cv::cvtColor(m_frame, m_frame, CV_YUV2RGB_UYVY);
  //mutex.unlock();


  out_msg.image = m_frame;
  image_pub_.publish(out_msg.toImageMsg());

}
