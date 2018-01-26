#include<stereo_viewer/viewer.h>
#include<ros/ros.h>
#include<string>
#include<iostream>

int main(int argc, char **argv){


	ros::init(argc, argv, "stereo_viewer");
	ros::NodeHandle nh;


  Viewer view(nh, "stereo/master/left/image_raw", 0,0, "stereo/master/right/image_raw", 3300, 0);

  //view.disp_checkerboard = true;

  view.setFilePath("/home/arclab");
  view.run();
  while(ros::ok()){}
	
	return 0;
}
