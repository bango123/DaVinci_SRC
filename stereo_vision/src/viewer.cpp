#include <stereo_vision/viewer.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <stdio.h>

Viewer::Viewer (ros::NodeHandle nh, const std::string& image_topic1, int pos_x1, int pos_y1,  const std::string& image_topic2, int pos_x2, int pos_y2) :
  m_nh(nh),
  m_it(m_nh), 
  m_image_topic1(image_topic1),
  m_image_topic2(image_topic2),
  m_running(false),
  new_img(false),
  m_disparity_set_up(false),
  m_disp_disparity(false),
  m_saveImagePair(false),
  m_imageNumberSaved(0),
  m_filePath("~/"),
  m_rows(0),
  m_cols(0),
  m_pos_x1(pos_x1),
  m_pos_y1(pos_y1),
  m_pos_x2(pos_x2),
  m_pos_y2(pos_y2)
{

  cv::namedWindow(m_image_topic1.c_str(), cv::WINDOW_NORMAL);

  cv::moveWindow(m_image_topic1.c_str(), pos_x1, pos_y1);
  cv::setWindowProperty(m_image_topic1.c_str(), cv::WND_PROP_FULLSCREEN, 1);

  cv::namedWindow(m_image_topic2.c_str(), cv::WINDOW_NORMAL);

  cv::moveWindow(m_image_topic2.c_str(), pos_x2, pos_y2);
  cv::setWindowProperty(m_image_topic2.c_str(), cv::WND_PROP_FULLSCREEN, 1);

  cv::waitKey(10);

  cv::setMouseCallback(m_image_topic1.c_str(), &Viewer::mouse_callback, this);
  cv::setMouseCallback(m_image_topic2.c_str(), &Viewer::mouse_callback, this);

  //Set up window 1 and subscriber 1
  m_sub1 = m_it.subscribe(m_image_topic1, 1, &Viewer::display_callback1, this);
  //Set up window 2 and subscriber 2
  m_sub2 = m_it.subscribe(m_image_topic2, 1, &Viewer::display_callback2, this);

}

Viewer::~Viewer(){
  cv::destroyWindow(m_image_topic1);
  cv::destroyWindow(m_image_topic2);

  if(m_disp_disparity){
     cv::destroyWindow("Disparity");
  }
}

void Viewer::run(){

	m_running = true;

	loop();

  m_running = false;
	return;
}

void Viewer::setDisparitySubscribers(const std::string& disp_info_topic){
  m_sub_disparity = m_nh.subscribe(disp_info_topic, 1, &Viewer::disparity_callback, this);
  m_disparity_set_up = true;
}

void Viewer::displayDisparity(bool disp){
  //Check if the cameraInfoSubscribers has been set up
  if(!m_disparity_set_up){
    return;
  }

  m_disp_disparity = disp;

  if(disp){
    cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
  }

  else{
    cv::destroyWindow("Disparity");
  }
}

void Viewer::setFilePath(const std::string& filePath){
  m_filePath = filePath;
  if(m_filePath.back() != '/'){
    m_filePath = m_filePath + '/';
  }

}

void Viewer::setResolution(int cols, int rows){
    m_rows = rows;
    m_cols = cols;
}

void Viewer::loop(){

    ros::Rate loop_rate(100);

	while(ros::ok()){
		ros::spinOnce();		

        if( m_img1.empty() || m_img2.empty()){
          continue;
        }


        //If there is no new images arrived, do nothing!!
        if( !new_img ){
          continue;
        }

        //Check to see if the timestamps are equal
        if( m_img1_ts != m_img2_ts){
            continue;
        }
        new_img = false;

        //Move the window.. This fixes bug!!
        cv::moveWindow(m_image_topic1.c_str(), m_pos_x1, m_pos_y1);
        cv::moveWindow(m_image_topic2.c_str(), m_pos_x2, m_pos_y2);


        //Check to see if we should save image pair
        if(m_saveImagePair){
            m_saveImagePair = false;
            saveImagePair(m_img1, m_img2,
                          m_filePath + "cam1_" + std::to_string(m_imageNumberSaved) + ".png",
                          m_filePath + "cam2_" + std::to_string(m_imageNumberSaved) + ".png");
            m_imageNumberSaved++;

        }

        //ROI the images based on the set resolution:
        cv::Rect roi1;
        if(m_rows < m_img1.rows && m_rows != 0){
            roi1.y = (m_img1.rows - m_rows)/2;
            roi1.height = m_rows;
        }
        else{
            roi1.y = 0;
            roi1.height = m_img1.rows;
        }

        if(m_cols < m_img1.cols && m_cols != 0){
            roi1.x = (m_img1.cols - m_cols)/2;
            roi1.width = m_cols;
        }
        else{
            roi1.x = 0;
            roi1.width = m_img1.cols;
        }

        cv::Rect roi2;
        if(m_rows < m_img2.rows && m_rows != 0){
            roi2.y = (m_img2.rows - m_rows)/2;
            roi2.height = m_rows;
        }
        else{
            roi2.y = 0;
            roi2.height = m_img2.rows;
        }

        if(m_cols < m_img2.cols && m_cols != 0){
            roi2.x = (m_img2.cols - m_cols)/2;
            roi2.width = m_cols;
        }
        else{
            roi2.x = 0;
            roi2.width = m_img2.cols;
        }

        cv::imshow( m_image_topic1, m_img1(roi1));
        cv::imshow( m_image_topic2, m_img2(roi2));

        //Display disparity
        if(m_disp_disparity && !m_disparity.empty()){
          cv::imshow( "Disparity", m_disparity);
        }

        cv::waitKey(1);
        loop_rate.sleep();
	}

    std::cout << "Exiting loop!!" << std::endl;

	return;
}

void Viewer::display_callback1(const sensor_msgs::ImageConstPtr & msg){
  if(!m_running){
    return;
  }
  new_img = true;

  m_img1_ts = msg->header.stamp;

  cv_ptr1 = cv_bridge::toCvShare(msg);
  m_img1 = cv_ptr1->image;

  cv::cvtColor(m_img1, m_img1, cv::COLOR_BGR2RGB);
  return;
}

void Viewer::display_callback2(const sensor_msgs::ImageConstPtr & msg){
  if(!m_running){
    return;
  }
  new_img = true;

  m_img2_ts = msg->header.stamp;

  cv_ptr2 = cv_bridge::toCvShare(msg);
  m_img2 = cv_ptr2->image;

  cv::cvtColor(m_img2, m_img2, cv::COLOR_BGR2RGB);
  return;
}

void Viewer::disparity_callback(const stereo_msgs::DisparityImage& msg){
  if(!m_running){
    return;
  }

  cv_ptr_disp = cv_bridge::toCvCopy(msg.image);
  m_disparity = cv_ptr_disp->image;

  return;
}

void Viewer::mouse_callback(int event, int x, int y, int flags, void* userdata){
  if(!userdata || event != cv::EVENT_LBUTTONDOWN ){
    return;
  }
   Viewer *viewer = (Viewer*) userdata;
   viewer->mouse_callback();
}

void Viewer::mouse_callback(){
    m_saveImagePair = true;
    return;
}

void Viewer::saveImagePair(const cv::Mat &img1, const cv::Mat &img2, const std::string& filename1, const std::string&filename2){
  std::cout << "---Saving Images---" << std::endl;
  if(cv::imwrite(filename1, img1)){
    std::cout << "Saved image " << filename1 << std::endl;
  }
  else{
    std::cout << "Failed to save image " << filename1 << std::endl;
  }

  if(cv::imwrite(filename2, img2)){
    std::cout << "Saved image " << filename2 << std::endl;
    if( img1.empty()){
      std::cout << "Image is empty" << std::endl;
    }
  }
  else{
    std::cout << "Failed to save image " << filename2 << std::endl;
    if( img2.empty()){
      std::cout << "Image is empty" << std::endl;
    }
  }

  cv::waitKey(30);

}
