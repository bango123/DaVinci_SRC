#ifndef PUBLISHIMAGEWORKER_H
#define PUBLISHIMAGEWORKER_H

#include <QThread>
#include <QMutex>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>


class PublishImageWorker : public QThread
{
  Q_OBJECT

public:
    //delay here is the amount of delay being added before publishing in ms.
    PublishImageWorker( bool isLeftCamera, ros::NodeHandle nh);
    ~PublishImageWorker(){}

    void setFrame(const cv::Mat frame, const std_msgs::Header header);

public slots:
    virtual void run() override;

signals:
    void finished();
    void error(QString err);

private:
    ros::NodeHandle                         nh_;
    image_transport::ImageTransport         it_;
    image_transport::CameraPublisher        image_pub_;
    camera_info_manager::CameraInfoManager  cinfo_;



   bool                             m_isLeftCamera;
   cv::Mat                          m_frame;
   std_msgs::Header                 m_header;

   QMutex mutex;
};

#endif // DECKLINKCAPTUREWORKER_H
