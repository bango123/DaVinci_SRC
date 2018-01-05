#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cisst_msgs/FloatStamped.h>



int main(int argc, char **argv)
{

  //Initialize ros
  ros::init(argc, argv, "test_dvrk");
  ros::NodeHandle nh;


  ros::Publisher psm1_jaw_pub  = nh.advertise <cisst_msgs::FloatStamped>   ("dvrk/PSM1/set_jaw_position" , 1);
  ros::Publisher psm1_cart_pub = nh.advertise <geometry_msgs::PoseStamped> ("dvrk/PSM1/set_position_cartesian" , 1);

  //Set to 1Hz for the while loop
  ros::Rate loop_rate(1);

  int count = 0;

  while (ros::ok())
  {
     cisst_msgs::FloatStamped   jaw_msg;
     geometry_msgs::PoseStamped pos_msg;

     jaw_msg.header.stamp = ros::Time::now();
     pos_msg.header.stamp = ros::Time::now();

     if( count % 2 == 0){
        jaw_msg.data = 0.9;

        pos_msg.pose.position.x = -0.05;
        pos_msg.pose.position.y = 0.04;
        pos_msg.pose.position.z = -0.074;

        pos_msg.pose.orientation.x =  0.869;
        pos_msg.pose.orientation.y = -0.198;
        pos_msg.pose.orientation.z = -0.278;
        pos_msg.pose.orientation.w = -0.357;
     }
     else{
        jaw_msg.data = 0.1;

        pos_msg.pose.position.x = -0.048;
        pos_msg.pose.position.y = 0.04;
        pos_msg.pose.position.z = -0.074;

        pos_msg.pose.orientation.x =  0.869;
        pos_msg.pose.orientation.y = -0.198;
        pos_msg.pose.orientation.z = -0.278;
        pos_msg.pose.orientation.w = -0.357;
     }
     psm1_jaw_pub.publish(jaw_msg);
     psm1_cart_pub.publish(pos_msg);

     count++;
     ros::spinOnce();
     loop_rate.sleep();
  }

  return 0;
}
