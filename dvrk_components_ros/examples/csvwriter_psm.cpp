#include <ros/ros.h>

#include <dvrk_components_ros/psm.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <fstream>
#include <sstream>


std::ofstream psm1_slave_file;
std::ofstream psm2_slave_file;

std::ofstream psm1_master_file;
std::ofstream psm2_master_file;

std::string getStringForCSV(geometry_msgs::PoseStamped msg){

    if(msg.header.stamp.sec == 0){
        return "";
    }

    std::stringstream stream;

    stream << msg.header.stamp       << ",";
    stream << msg.pose.position.x    << "," << msg.pose.position.y    << "," << msg.pose.position.z    << ",";
    stream << msg.pose.orientation.x << "," << msg.pose.orientation.y << "," << msg.pose.orientation.z << ",";
    stream << msg.pose.orientation.w << "\n";

    return stream.str();
}

int main(int argc, char **argv)
{
  //Initialize ros
  ros::init(argc, argv, "csvwriter_psm");
  ros::NodeHandle nh;

  //Initialize PSM components
  PSM psm1(nh,1);
  PSM psm2(nh,2);

  if(argc != 2){
      std::cout << "The only input paramter should be path to folder where CSV files will be written" << std::endl;
      return 0;
  }

  std::string folderPath = argv[1];
  if (folderPath.back() != '/'){
      folderPath.push_back('/');
  }

  //Open up all the files
  std::string psm1_slave_file_str  = folderPath + "psm1_slave_data.csv";
  std::string psm2_slave_file_str  = folderPath + "psm2_slave_data.csv";
  std::string psm1_master_file_str = folderPath + "psm1_master_data.csv";
  std::string psm2_master_file_str = folderPath + "psm2_master_data.csv";

  std::cout << "Writing to " << psm1_slave_file_str  << std::endl;
  std::cout << "Writing to " << psm2_slave_file_str  << std::endl;
  std::cout << "Writing to " << psm1_master_file_str << std::endl;
  std::cout << "Writing to " << psm2_master_file_str << std::endl;

  psm1_slave_file.open(psm1_slave_file_str);
  psm2_slave_file.open(psm2_slave_file_str);

  psm1_master_file.open(psm1_master_file_str);
  psm2_master_file.open(psm2_master_file_str);

  //Add header to all the csv files
  std::string header = "TimeStamp, Position_x, Position_y, Position_z, Orientation_x, Orientation_y, Orientation_z, Orientation_w\n";

  psm1_slave_file  << header;
  psm2_slave_file  << header;
  psm1_master_file << header;
  psm2_master_file << header;

  //Set ros loop rate to 1Hz
  ros::Rate loop_rate(100);

  while(ros::ok()){
      psm1_slave_file  << getStringForCSV(psm1.get_slave_cart_pos());
      psm2_slave_file  << getStringForCSV(psm2.get_slave_cart_pos());
      psm1_master_file << getStringForCSV(psm1.get_master_cart_pos());
      psm2_master_file << getStringForCSV(psm2.get_master_cart_pos());

   ros::spinOnce();
    loop_rate.sleep();
  }

  //Close all files:
    psm1_slave_file.close();
    psm2_slave_file.close();

    psm1_master_file.close();
    psm2_master_file.close();

    return 0;
}
