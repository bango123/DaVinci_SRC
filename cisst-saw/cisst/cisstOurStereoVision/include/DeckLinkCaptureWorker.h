#ifndef DECKLINKCAPTUREWORKER_H
#define DECKLINKCAPTUREWORKER_H

#include <QThread>
#include <QMutex>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>


class DeckLinkCaptureWorker : public QThread
{
  Q_OBJECT

public:
    DeckLinkCaptureWorker( bool isLeftCamera);
    ~DeckLinkCaptureWorker(){}

    void setFrame(const cv::Mat frame, const std_msgs::Header header);

public slots:
    virtual void run() override;

signals:
    void finished();
    void error(QString err);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;



   bool                             m_isLeftCamera;
   cv::Mat                          m_frame;
   std_msgs::Header                 m_header;

   QMutex mutex;
};

#endif // DECKLINKCAPTUREWORKER_H
