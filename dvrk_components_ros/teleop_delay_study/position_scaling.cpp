#include <ros/ros.h>
#include <dvrk_components_ros/psm.h>
#include <dvrk_components_ros/teleop.h>

#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

std::vector<float> post1_psm1;
std::vector<float> post2_psm1;
std::vector<float> post3_psm1;
std::vector<float> post4_psm1;

std::vector<float> post1_psm2;
std::vector<float> post2_psm2;
std::vector<float> post3_psm2;
std::vector<float> post4_psm2;

const float LIMIT = 0.015;
float WEIGHTXY = 1.0;
float WEIGHTZ = 1.0;
float MINSCALE = 0.5;
float MAXSCALE = 1.0;

//Used to parse a line from the cal file
std::vector<float> commaString2Vect(std::string line){
    std::vector<float> vect;
    std::stringstream ss(line);

    float i;
    while (ss >> i)
    {
        vect.push_back(i);

        if (ss.peek() == ',')
            ss.ignore();
    }

    return vect;
}

float distance2post(geometry_msgs::PoseStamped pose, std::vector<float> post){

    float dx = pose.pose.position.x-post[0];
    float dy = pose.pose.position.y-post[1];
    //float dz = pose.pose.position.z-post[2];

    return sqrt(dx*dx + dy*dy);
}


float minDistanceFromPosts(geometry_msgs::PoseStamped pose, std::vector<float> postArray[], int numPosts){
    float minDistance = distance2post( pose, postArray[0]);
    for(int i = 1; i < numPosts; i++){
          float temp =  distance2post( pose, postArray[i]);
          if(temp < minDistance){
              minDistance = temp;
          }
    }
    return minDistance;
}

float directionDistance(float dx, float dy, geometry_msgs::PoseStamped prev_pose, std::vector<float> post){
    float x1 = prev_pose.pose.position.x + dx;
    float y1 = prev_pose.pose.position.y + dy;
    float x2 = x1 + y1 - prev_pose.pose.position.y;
    float y2 = y1 + x1 - prev_pose.pose.position.x;
    float x0 = post[0];
    float y0 = post[1];
    return ((y2 - y1) * x0 + (x2 - x1) * y0 + x2 * y1 - y2 * x1) / sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
}

float minDirectionDistance(float dx, float dy, geometry_msgs::PoseStamped prev_pose, std::vector<float> postArray[], int numPosts){
    float minDistance = directionDistance(dx, dy, prev_pose, postArray[0]);
    for(int i = 1; i < numPosts; i++){
          float temp =  directionDistance(dx, dy, prev_pose, postArray[i]);
          if(temp < minDistance){
              minDistance = temp;
          }
    }
    return minDistance;
}

int main(int argc, char **argv)
{
    if(argc != 6){
        std::cout << "First argument must be float and is weight1" << std::endl;
        std::cout << "Second argument must be float and is weight2" << std::endl;
        std::cout << "you a clown. You missing minscale" << std::endl;
        std::cout << "do we really even need this message?" << std::endl;
        std::cout << "why don't you add an error message that actually tells you something?" << std::endl;
        std::cout << "why don't you at least add zbonus there? THat's the point of these error messages..." << std::endl;
        return 0;
    }
    WEIGHTXY = atof(argv[1]);
    WEIGHTZ = atof(argv[2]);
    MINSCALE = atof(argv[3]);
    MAXSCALE = atof(argv[4]);
    float zbonus = atof(argv[5]);
//    bool zachisstooppid = atoi(argv[6]);

    std::string calFile_str = "/home/arclab/catkin_ws/src/dvrk_components_ros/teleop_delay_study/calFile.txt";

    std::ifstream  calFile;
    calFile.open(calFile_str);

    //Parse the cal file:
    std::string line;

    //First line reads PSM1:
    std::getline( calFile, line );
    std::getline( calFile, line );
    post1_psm1 = commaString2Vect(line);
    std::getline( calFile, line );
    post2_psm1 = commaString2Vect(line);
    std::getline( calFile, line );
    post3_psm1 = commaString2Vect(line);
    std::getline( calFile, line );
    post4_psm1 = commaString2Vect(line);

    std::cout << "For PSM1 post values are: " << std::endl;
    std::cout << post1_psm1[0] << "," << post1_psm1[1] << "," << post1_psm1[2] << std::endl;
    std::cout << post2_psm1[0] << "," << post2_psm1[1] << "," << post2_psm1[2] << std::endl;
    std::cout << post3_psm1[0] << "," << post3_psm1[1] << "," << post3_psm1[2] << std::endl;
    std::cout << post4_psm1[0] << "," << post4_psm1[1] << "," << post4_psm1[2] << std::endl;

    //First line reads PSM2:
    std::getline( calFile, line );
    std::getline( calFile, line );
    post1_psm2 = commaString2Vect(line);
    std::getline( calFile, line );
    post2_psm2 = commaString2Vect(line);
    std::getline( calFile, line );
    post3_psm2 = commaString2Vect(line);
    std::getline( calFile, line );
    post4_psm2 = commaString2Vect(line);


    std::cout << "For PSM2 post values are: " << std::endl;
    std::cout << post1_psm2[0] << "," << post1_psm2[1] << "," << post1_psm2[2] << std::endl;
    std::cout << post2_psm2[0] << "," << post2_psm2[1] << "," << post2_psm2[2] << std::endl;
    std::cout << post3_psm2[0] << "," << post3_psm2[1] << "," << post3_psm2[2] << std::endl;
    std::cout << post4_psm2[0] << "," << post4_psm2[1] << "," << post4_psm2[2] << std::endl;

    calFile.close();

    //Initialize ros
    ros::init(argc, argv, "test_psm");
    ros::NodeHandle nh;

    //Initialize teleop component
    Teleop teleop(nh);

    //Initialize PSM components
    PSM psm1(nh,1);
    PSM psm2(nh,2);

    std::cout << "---  Reseting DVRK  ---" << std::endl;
    if(!teleop.reset_dvrk())
    {
      return 0;
    }
    std::cout << "---Turning on System---" << std::endl;

    if(!teleop.turn_onTeleop()){
      return 0;
    }
    std::cout << "--- Turning on Ros  ---" << std::endl;

    if(!teleop.set_ros_only_wait(true)){
      return 0;
    }


    std::vector<float> postArray_psm1[4] = {post1_psm1, post2_psm1, post3_psm1, post4_psm1};
    std::vector<float> postArray_psm2[4] = {post1_psm2, post2_psm2, post3_psm2, post4_psm2};

    //This is to keep track of the last target position
    geometry_msgs::PoseStamped last_target1;
    geometry_msgs::PoseStamped last_target2;

    //Tracking the actual position we are sending to the arms
    geometry_msgs::PoseStamped target_sent1;
    geometry_msgs::PoseStamped target_sent2;

    float scale = 0.5;

    std::cout << "Ready to go!!!!" << std::endl;

    //Passthrough code
    bool firstTime = true;
    int firstTimeCounter = 0;

    while(ros::ok()){

      ros::spinOnce();

      if(teleop.get_operatorPresent() && teleop.get_powerStatus() && teleop.get_teleopStatus()){

        geometry_msgs::PoseStamped new_target1 = psm1.get_target_slave_cart_pos();
        geometry_msgs::PoseStamped new_target2 = psm2.get_target_slave_cart_pos();

        //For first time!
        if( firstTime ){
            last_target1 = new_target1;
            last_target2 = new_target2;

            target_sent1 = new_target1;
            target_sent2 = new_target2;

            //use passthrough for the first few iterations
            psm1.set_master_cart_pos(psm1.get_target_master_cart_pos());
            psm1.set_master_jaw(psm1.get_target_master_jaw());
            psm1.set_slave_jaw(psm1.get_target_slave_jaw());
            psm1.set_slave_cart_pos(psm1.get_target_slave_cart_pos());

            psm2.set_master_cart_pos(psm2.get_target_master_cart_pos());
            psm2.set_master_jaw(psm2.get_target_master_jaw());
            psm2.set_slave_jaw(psm2.get_target_slave_jaw());
            psm2.set_slave_cart_pos(psm2.get_target_slave_cart_pos());

            firstTimeCounter++;
            if(firstTimeCounter > 50){
                firstTime = false;
            }
            continue;
        }

        //Don't do anything if no new data has been received
        if( last_target1.header.stamp == new_target1.header.stamp || last_target2.header.stamp == new_target2.header.stamp){
            continue;
        }

        geometry_msgs::PoseStamped actualPose_1 = psm1.get_slave_cart_pos();
        geometry_msgs::PoseStamped actualPose_2 = psm2.get_slave_cart_pos();

        float dx1  =  new_target1.pose.position.x - last_target1.pose.position.x;
        float dy1  =  new_target1.pose.position.y - last_target1.pose.position.y;
        float dz1  =  new_target1.pose.position.z - last_target1.pose.position.z;

        float dx2  =  new_target2.pose.position.x - last_target2.pose.position.x;
        float dy2  =  new_target2.pose.position.y - last_target2.pose.position.y;
        float dz2  =  new_target2.pose.position.z - last_target2.pose.position.z;

        if( dx1*dx1 + dy1*dy1 + dz1*dz1 > 0.000005 ||  dx2*dx2 + dy2*dy2 + dz2*dz2 > 0.000005){
            std::cout << "A jerk occured" << std::endl;

            last_target1 = new_target1;
            last_target2 = new_target2;
            continue;

        }

        float minDist_1 = (minDistanceFromPosts(actualPose_1, postArray_psm1, 4) - LIMIT)>0.0 ? (minDistanceFromPosts(actualPose_1, postArray_psm1, 4) - LIMIT): 0.0;
        float minDist_2 = (minDistanceFromPosts(actualPose_2, postArray_psm2, 4) - LIMIT)>0.0 ? (minDistanceFromPosts(actualPose_2, postArray_psm2, 4) - LIMIT): 0.0;

//        if(zachisstooppid){
//            minDist_1 = minDirectionDistance(dx1, dy1, actualPose_1, postArray_psm1, 4);
//            minDist_2 = minDirectionDistance(dx2, dy2, actualPose_2, postArray_psm2, 4);

//        }
//        std::cout << "PSM1 distance from closest post: " << minDist_1 << std::endl;
//        std::cout << "PSM2 distance from closest post: " << minDist_2 << std::endl;

        //scale math
        float scaleXY1 = minDist_1 * WEIGHTXY + MINSCALE >MAXSCALE ? MAXSCALE : minDist_1 * WEIGHTXY + MINSCALE;
        float scaleXY2 = minDist_2 * WEIGHTXY + MINSCALE >MAXSCALE ? MAXSCALE : minDist_2 * WEIGHTXY + MINSCALE;
        float scaleZ1 = 1.0;
        float scaleZ2 = 1.0;


        float zavg1 = (post1_psm1[2] + post2_psm1[2] + post3_psm1[2] + post4_psm1[2]) / 4;
        float zavg2 = (post1_psm2[2] + post2_psm2[2] + post3_psm2[2] + post4_psm2[2]) / 4;
        float zavg = (zavg1 > zavg2) ? zavg1 : zavg2;

        scaleXY1 += (actualPose_1.pose.position.z > zavg) ? zbonus : 0.0;
        scaleXY2 += (actualPose_2.pose.position.z > zavg) ? zbonus : 0.0;


//        float apdx = actualPose_1.pose.position.x - actualPose_2.pose.position.x;
//        float apdy = actualPose_1.pose.position.y - actualPose_2.pose.position.y;
//        float apdz = actualPose_1.pose.position.z - actualPose_2.pose.position.z;

//        if ((apdx * apdx + apdy * apdy + apdz * apdz) < (2 * LIMIT * 2 * LIMIT * 2)) {
//            scaleXY1 *= 0.5;
//            scaleXY2 *= 0.5;
//        }

//        std::cout<< "XY scale:  "<<scaleXY1*0.2<<std::endl;
        //Apply scaling  
        target_sent1.pose.position.x  += scaleXY1*( dx1 );
        target_sent1.pose.position.y  += scaleXY1*( dy1 );
        target_sent1.pose.position.z  += scaleZ1*( dz1 );
        target_sent1.pose.orientation  = new_target1.pose.orientation;

        target_sent2.pose.position.x  += scaleXY2*( dx2 );
        target_sent2.pose.position.y  += scaleXY2*( dy2 );
        target_sent2.pose.position.z  += scaleZ2*( dz2 );
        target_sent2.pose.orientation  = new_target2.pose.orientation;



        //Send out the commands
        psm1.set_master_cart_pos(psm1.get_target_master_cart_pos());
        psm1.set_master_jaw(psm1.get_target_master_jaw());
        psm1.set_slave_jaw(psm1.get_target_slave_jaw());
        psm1.set_slave_cart_pos(target_sent1);

        psm2.set_master_cart_pos(psm2.get_target_master_cart_pos());
        psm2.set_master_jaw(psm2.get_target_master_jaw());
        psm2.set_slave_jaw(psm2.get_target_slave_jaw());
        psm2.set_slave_cart_pos(target_sent2);


        last_target1 = new_target1;
        last_target2 = new_target2;
      }

    }


}
