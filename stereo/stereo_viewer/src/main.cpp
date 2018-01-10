#include<stereo_viewer/viewer.h>
#include<ros/ros.h>
#include<string>
#include<iostream>

int main(int argc, char **argv){


	ros::init(argc, argv, "stereo_viewer");
	ros::NodeHandle nh;


  Viewer view(nh, "stereo/master/left/image_raw", 0,0, "stereo/master/right/image_raw", 3300, 0);

  view.run();
  while(ros::ok()){}
	
//	std::string cam_param;

//	if( argc != 2){
//		std::cout << "Please specify L or R cam" << std::endl;
//		return 0;
//	}
//	cam_param = argv[1];
		
//	if( !cam_param.compare("L") ){
//		std::cout << "Displaying left camera on left display" << std::endl;
//		Viewer view(nh, "stereo/master/left/image_raw", 0,0);
//		view.run();
//		while(ros::ok()){}

//	}
//	else if ( !cam_param.compare("R")){
//		std::cout << "Displaying right camera on right display" << std::endl;
//		Viewer view(nh, "stereo/master/right/image_raw", 3300, 0);
//		view.run();
//		while(ros::ok()){}
//	}
//	else{
//		std::cout << "Please specify L or R cam" << std::endl;
//		return 0;
//	}

	
	return 0;
}
