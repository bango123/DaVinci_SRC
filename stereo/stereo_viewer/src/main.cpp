#include<stereo_viewer/viewer.h>
#include<ros/ros.h>
#include<string>
#include<iostream>

int main(int argc, char **argv){


	ros::init(argc, argv, "stereo_viewer");
	ros::NodeHandle nh;


  Viewer view(nh, "stereo/master/left/image_raw", 0,0, "stereo/master/right/image_raw", 3300, 0);

  //view.disp_checkerboard = true;
  view.setResolution(1400, 1050);

  view.setFilePath("/home/arclab");
  view.run();


  ros::Rate loop_rate(1);
  while (ros::ok()){
      ros::spinOnce();
      loop_rate.sleep();
  }
	
	return 0;
}
