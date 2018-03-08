#include <stereo_vision/PublishImageWorker.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <cv_bridge/cv_bridge.h>

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


PublishImageWorker::PublishImageWorker( bool isLeftCamera, ros::NodeHandle nh) :
    m_isLeftCamera(isLeftCamera),
    nh_(nh),
    cinfo_(nh_),
    it_(nh_),
    image_pub_( it_.advertiseCamera("image_raw", 1) )
{
  if(m_isLeftCamera){
    cinfo_.setCameraName("left");
  }
  else{
    cinfo_.setCameraName("right");
  }

  cinfo_.loadCameraInfo("file://${ROS_HOME}/camera_info/${NAME}.yaml");

  signal(SIGSEGV, handler);
}


void PublishImageWorker::setFrame(const cv::Mat frame, const std_msgs::Header header){

  m_frame = frame;
  m_header = header;
}

void PublishImageWorker::run() {
  if(!ros::ok()){
    return;
  }

  cv_bridge::CvImage out_msg;
  out_msg.header   = m_header;
  out_msg.encoding = sensor_msgs::image_encodings::RGB8;

  cv::cvtColor(m_frame, m_frame, CV_YUV2RGB_UYVY);

  out_msg.image = m_frame;

  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_.getCameraInfo()));

  ci->header.frame_id = out_msg.header.frame_id;
  ci->header.stamp    = out_msg.header.stamp;

  ci->height = m_frame.rows;
  ci->width  = m_frame.cols;

  image_pub_.publish(out_msg.toImageMsg(), ci);
  m_frame.release();
}
