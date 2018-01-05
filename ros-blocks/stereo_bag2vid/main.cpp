#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


//Done in the tutorial. Used when parsing ros bag
#define foreach BOOST_FOREACH

//FPS targetting!!!
int fps = 30;


//THIS ASSUMES THAT THE LEFT AND RIGHT IMAGE STREAMS ARE SYNCHRONIZED!!! SO THEY MUST HAVE THE SAME HEADER TIMESTAMP.
int main(int argc, char **argv){
	if( argc != 3) {
		std::cout << "Please specify path to folder that includes .bag cam file from DVRK and specify output file (include .avi)" << std::endl;
		return 0;
	}

	//Adds / if there isn't one to the end of the folder path. Helps for later!
	std::string folderPath = argv[1];
	if (folderPath.back() != '/'){
		folderPath.push_back('/');
	}
	std::cout << "Using path " << folderPath << std::endl;

	boost::filesystem::path p(folderPath);

	//Need to order the files in the directory	
	boost::filesystem::directory_iterator end_itr_dir;
	std::list<std::string> 	file_list; // list of filepaths in order!!

	int fileNumber = 0;
	for( boost::filesystem::directory_iterator itr_dir(p); itr_dir != end_itr_dir; itr_dir++) {
			for( boost::filesystem::directory_iterator itr_search(p); itr_search != end_itr_dir; itr_search++){		
				std::string currentFile = itr_search->path().string();
				std::size_t start_index = currentFile.rfind("_");
				std::size_t end_index   = currentFile.rfind(".");

				if( start_index == std::string::npos || end_index == std::string::npos){
					continue;
				}
				int currentFileNumber = std::stoi(currentFile.substr(start_index+1, end_index-start_index-1));
				if(currentFileNumber == fileNumber){	
					file_list.push_back(currentFile);					
				}
			}
		fileNumber++;
	}
	
	double period = 1.0/(double(fps));

	rosbag::Bag bag;

	//Vector topics contains the topics we are interested in when using rosbag play
	std::vector<std::string> topics;	
	std::string l_topic = "/stereo/slave/left/image_raw";
	std::string r_topic = "/stereo/slave/right/image_raw";

	topics.push_back(l_topic);
	topics.push_back(r_topic);

	//Current set of images + timestamps
	cv::Mat leftImg;
	ros::Time leftImg_ts;

	cv::Mat rightImg;
	ros::Time rightImg_ts;

	//Previous set of images + time stamps, used for filling in dropped frames
	cv::Mat last_leftImg;
	cv::Mat last_rightImg;
	ros::Time last_ts(0.0);
	ros::Time first_ts(0.0);

	bool firstIteration = true;
	int numberOfDroppedFrames = 0;
	int numberOfFrames = 0;
	
	//Mat used to hold both left and right. Will be the output
	cv::Mat combinedImage;



	//Going to open a bag quickly to get the resolution for the output video!!!
	cv::Size outputResolution;
	
	bag.open(*file_list.begin(), rosbag::bagmode::Read);
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	
	foreach(rosbag::MessageInstance const m, view){

		if(m.getTopic() == l_topic || "/" + m.getTopic() == l_topic){
			sensor_msgs::Image::ConstPtr currentL_msg = m.instantiate<sensor_msgs::Image>();
			cv_bridge::CvImageConstPtr currentL_cvBridge = cv_bridge::toCvShare(currentL_msg);
	
			outputResolution.height = currentL_cvBridge->image.rows;
			outputResolution.width  = currentL_cvBridge->image.cols*2;
			break;
		}
	}
	bag.close();

	cv::VideoWriter outVideo(argv[2] ,CV_FOURCC('M','J','P','G'), fps, outputResolution, true);

	//cv::namedWindow("test", cv::WINDOW_NORMAL);
	
	for(std::list<std::string>::const_iterator file_itr = file_list.begin(); file_itr != file_list.end(); file_itr++){
		std::cout << "On file: " << *file_itr << std::endl;
		//open the bag
		bag.open(*file_itr, rosbag::bagmode::Read);
		
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		foreach(rosbag::MessageInstance const m, view){
			
			if( m.getTopic() == l_topic || "/" + m.getTopic() == l_topic){
				sensor_msgs::Image::ConstPtr currentL_msg = m.instantiate<sensor_msgs::Image>(); 
				
				cv_bridge::CvImageConstPtr currentL_cvBridge = cv_bridge::toCvShare(currentL_msg);				
				cv::cvtColor(currentL_cvBridge->image, leftImg,  cv::COLOR_BGR2RGB);
				leftImg_ts = currentL_msg->header.stamp;
				//std::cout << "Left image ts:  " << leftImg_ts << std::endl;
			}

			if( m.getTopic() == r_topic || "/" + m.getTopic() == r_topic){
				sensor_msgs::Image::ConstPtr currentR_msg = m.instantiate<sensor_msgs::Image>();
				cv_bridge::CvImageConstPtr currentR_cvBridge = cv_bridge::toCvShare(currentR_msg);
				cv::cvtColor(currentR_cvBridge->image, rightImg,  cv::COLOR_BGR2RGB);
				rightImg_ts = currentR_msg->header.stamp;
				//std::cout << "Right Image ts: " << rightImg_ts << std::endl;
			}

			//For first iteration
			if(rightImg.empty() || leftImg.empty()){
				continue;
			}

			//If the time stamps for left and right image are not equal then continue
			if(rightImg_ts != leftImg_ts){
				continue;
			}

			//Now leftImg and rightImg have the same time stamp. 
			
			//Trick for for first iteratin in foreach/for loop. Basically we know the bag is recorded so the time would never
			//be greater than ros::time:now()
			if( firstIteration ){
				firstIteration = false;
				last_ts = rightImg_ts;
				first_ts = rightImg_ts;

				hconcat(leftImg, rightImg, combinedImage);				
				outVideo.write(combinedImage);				

				//cv::imshow("test", combinedImage);
				//cv::waitKey(33);
				numberOfFrames++;				

				last_leftImg = leftImg.clone();
				last_rightImg = rightImg.clone();
				continue;
			}
			
			//Fill in last images till we are all caught up
			while( (rightImg_ts - last_ts).toSec()/period > 1.5){
				last_ts = last_ts + ros::Duration(period);
				
				hconcat(last_leftImg, last_rightImg, combinedImage);
				//cv::imshow("test", combinedImage);		
				//cv::waitKey(33);
				outVideo.write(combinedImage);

				numberOfFrames++;
				numberOfDroppedFrames++;
			}
			
			//all caught up and use the new images!
			last_ts = rightImg_ts;
			hconcat(leftImg, rightImg, combinedImage);
			
			outVideo.write(combinedImage);
			//cv::imshow("test", combinedImage);
			//cv::waitKey(33);
			numberOfFrames++;

			last_leftImg = leftImg.clone();
			last_rightImg = rightImg.clone();
			
		}

		bag.close();
	}
	outVideo.release();
	std::cout << "----FINISHED----" << std::endl;
	std::cout << "Number of Frames Saved  : " << numberOfFrames << std::endl;
	std::cout << "Number of Dropped Frames: " << numberOfDroppedFrames << std::endl;
	std::cout << "First time stamp        : " << first_ts << std::endl;
	std::cout << "Last time stamp         : " << rightImg_ts << std::endl;
	return 0;
}
