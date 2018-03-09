#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>


//ROS Params:
// fps (default 30)
// filepath (default "output.avi")
// left_image
// right_image

// Example:
// rosrun stereo_ros_blocks stereo_videorecorder fps:=30 filepath:=/media/arclab/NVME/test.avi
// left_image:=/stereo/slave/left/image_raw right_image:=/stereo/slave/right/image_raw

//Contains the new image data
cv::Mat newImg_L;
cv::Mat newImg_R;
ros::Time newT_L;
ros::Time newT_R;

//Contains the last image data
cv::Mat oldImg_L;
cv::Mat oldImg_R;
ros::Time lastT_L;
ros::Time lastT_R;

bool newFrameArrived;

void imageCallback_L(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cvBridge = cv_bridge::toCvShare(msg);
    cv::cvtColor(cvBridge->image, newImg_L, cv::COLOR_BGR2RGB);
    newT_L = msg->header.stamp;

    newFrameArrived = true;
}

void imageCallback_R(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cvBridge = cv_bridge::toCvShare(msg);
    cv::cvtColor(cvBridge->image, newImg_R, cv::COLOR_BGR2RGB);
    newT_R = msg->header.stamp;

    newFrameArrived = true;
}

int main(int argc, char **argv){

    //Initialize ros
    ros::init(argc, argv, "stereo_video_recorder");
    ros::NodeHandle nh;

    int fps = 30;
    if(nh.hasParam("fps")){
        nh.getParam("fps", fps);

        std::cout << "FPS Set to: " << fps << std::endl;
    }
    double period = 1.0/(double(fps));


    std::string filepath = "output.avi";
    if(nh.hasParam("filepath")){
        nh.getParam("filepath", filepath);

        std::cout << "Output FilePath :" << filepath << std::endl;
    }

    std::string left_image_topic;
    if(nh.hasParam("left_image")){
        nh.getParam("left_image", left_image_topic);

        std::cout << "left image topic: " << left_image_topic << std::endl;
    }
    else{
        std::cout << "left_image param is requried to run this" << std::endl;
        return 0;
    }

    std::string right_image_topic;
    if(nh.hasParam("right_image")){
        nh.getParam("right_image", right_image_topic);

        std::cout <<  "right image topic: " << right_image_topic << std::endl;
    }
    else{
        std::cout << "right_image param is requried to run this" << std::endl;
        return 0;
    }

    //Set up to subscribe to the stereo camera feed
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_L = it.subscribe(left_image_topic  , 1, imageCallback_L);
    image_transport::Subscriber sub_R = it.subscribe(right_image_topic , 1, imageCallback_R);

    //Going to find first image pair here!!!
    ros::Rate loop_rate(60);
    while(ros::ok){
        //Just for the first case!
        if(!newImg_L.empty() && !newImg_R.empty()){
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    oldImg_R = newImg_R;
    oldImg_L = newImg_L;
    lastT_L  = newT_L;
    lastT_R  = newT_R;
    ros::Time firstT = newT_R;

    cv::Size outputResolution;

    outputResolution.height = newImg_L.rows;
    outputResolution.width  = 2*newImg_L.cols;

    //Set up video writer:
    cv::VideoWriter outVideo_L("L"+filepath,CV_FOURCC('M','J','P','G'), fps, cv::Size(newImg_L.cols, newImg_L.rows), true);
    cv::VideoWriter outVideo_R("R"+filepath,CV_FOURCC('M','J','P','G'), fps, cv::Size(newImg_L.cols, newImg_L.rows), true);

    cv::Mat combinedImage;

    //Keep track of stats
    int numberOfDroppedFrames = 0;
    int numberOfFrames = 0;

    newFrameArrived = false;


    while(ros::ok){
        ros::spinOnce();
        loop_rate.sleep();

        if(!newFrameArrived){
            continue;
        }

        //Should newT_Ld be synchronized data stream. So we should only process if there is a new pair
        if(newT_L != newT_R || (newT_L == lastT_L || newT_R == lastT_R)){
            continue;
        }


        //Fill in last images till we are all caught up
        while( (newT_L - lastT_L).toSec()/period > 1.5){
            lastT_L = lastT_L + ros::Duration(period);


            //hconcat(oldImg_L, oldImg_R, combinedImage);
            outVideo_R.write(oldImg_R);
            outVideo_L.write(oldImg_L);
            //outVideo.write(combinedImage);

            numberOfFrames++;
            numberOfDroppedFrames++;
        }

        //all caught up and use the new images!
        lastT_L = newT_L;
        lastT_R = newT_R;

        oldImg_R = newImg_R;
        oldImg_L = newImg_L;


        //hconcat(newImg_L, newImg_R, combinedImage);
        //cv::cvtColor(combinedImage, combinedImage,  cv::COLOR_BGR2RGB);

        //outVideo.write(combinedImage);
        outVideo_R.write(oldImg_R);
        outVideo_L.write(oldImg_L);
        numberOfFrames++;

        newFrameArrived = false;


    } //end ros::ok loop


    std::cout << "----FINISHED----" << std::endl;
    std::cout << "Number of Frames Saved  : " << numberOfFrames << std::endl;
    std::cout << "Number of Dropped Frames: " << numberOfDroppedFrames << std::endl;
    std::cout << "First time stamp        : " << firstT << std::endl;
    std::cout << "Last time stamp         : " << lastT_R << std::endl;

    //outVideo.release();
     outVideo_L.release();
}
