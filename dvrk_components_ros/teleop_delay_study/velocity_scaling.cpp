#include <ros/ros.h>
#include <dvrk_components_ros/psm.h>
#include <dvrk_components_ros/teleop.h>

#include <string>
#include <stdio.h>
#include <cmath>
#include <math.h>

//Weight values, see calcSpeedScale
float WEIGHT1 = 0.02;
float WEIGHT2 = 0.0; //2.0;
float SCALE_CAP = 1.0;

//HELPER METHODS
//
geometry_msgs::PoseStamped calcSpeedScale(geometry_msgs::PoseStamped old_master, geometry_msgs::PoseStamped new_master,  geometry_msgs::PoseStamped actual);

//MAIN
//
int main(int argc, char **argv)
{
    if(argc != 3){
        std::cout << "First argument must be float and is weight1" << std::endl;
        std::cout << "Second argument must be float and is weight2" << std::endl;
        return 0;
    }
    WEIGHT1 = atof(argv[1]);
    WEIGHT2 = atof(argv[2]);

    std::cout << "Using weight1 = " << WEIGHT1 << std::endl;
    std::cout << "Using weight2 = " << WEIGHT2 << std::endl;

    //Initialize ros
    ros::init(argc, argv, "velocity_scaling");
    ros::NodeHandle nh;

    //Initialize teleop component
    Teleop teleop(nh);

    //Initialize PSM components
    PSM psm1(nh,1);
    PSM psm2(nh,2);

    std::cout << "---  Reseting DVRK  ---" << std::endl;
    //Initializing DVRK
    if(!teleop.reset_dvrk()) {
        return 0;
    }

    std::cout << "---Turning on System---" << std::endl;
    //Turn on teleop
    if(!teleop.turn_onTeleop()) {
        return 0;
    }

    std::cout << "--- Turning on Ros  ---" << std::endl;
    //Turn on ros only
    if(!teleop.set_ros_only_wait(true)){
      return 0;
    }

    geometry_msgs::PoseStamped old_msg1;
    geometry_msgs::PoseStamped old_msg2;

    geometry_msgs::PoseStamped actual_target1;
    geometry_msgs::PoseStamped actual_target2;

    bool firstTime = true;
    int firstTimeCounter = 0;

    //ros::Rate loop_rate(1000);

    std::cout << "Ready to go!!!!" << std::endl;
    while(ros::ok())
    {
        ros::spinOnce();
        //loop_rate.sleep();

        if(teleop.get_operatorPresent() && teleop.get_powerStatus() 
            && teleop.get_teleopStatus()) {

            geometry_msgs::PoseStamped new_msg1 = psm1.get_target_master_cart_pos();
            geometry_msgs::PoseStamped new_msg2 = psm2.get_target_master_cart_pos();

            //For first time in loop
            if(firstTime){
                psm1.set_master_cart_pos(new_msg1);
                psm2.set_master_cart_pos(new_msg2);

                //passthrough for the jaw angle on master side
                psm1.set_master_jaw(psm1.get_target_master_jaw());
                psm2.set_master_jaw(psm2.get_target_master_jaw());

                //Passthrough for the slave side
                psm1.set_slave_jaw(psm1.get_target_slave_jaw());
                psm2.set_slave_jaw(psm2.get_target_slave_jaw());
                psm1.set_slave_cart_pos(psm1.get_target_slave_cart_pos());
                psm2.set_slave_cart_pos(psm2.get_target_slave_cart_pos());

                old_msg1 = new_msg1;
                old_msg2 = new_msg2;

                actual_target1 = old_msg1;
                actual_target2 = old_msg2;

                firstTimeCounter++;
                if(firstTimeCounter > 10){
                    firstTime = false;
                }
                continue;
            }

            float dx1  =  new_msg1.pose.position.x - old_msg1.pose.position.x;
            float dy1  =  new_msg1.pose.position.y - old_msg1.pose.position.y;
            float dz1  =  new_msg1.pose.position.z - old_msg1.pose.position.z;

            float dx2  =  new_msg2.pose.position.x - old_msg2.pose.position.x;
            float dy2  =  new_msg2.pose.position.y - old_msg2.pose.position.y;
            float dz2  =  new_msg2.pose.position.z - old_msg2.pose.position.z;

            if( dx1*dx1 + dy1*dy1 + dz1*dz1 > 0.000004 ||  dx2*dx2 + dy2*dy2 + dz2*dz2 > 0.000004){
                std::cout << "A jerk occured" << std::endl;

                old_msg1 = new_msg1;
                old_msg2 = new_msg2;
                continue;

            }
            if(old_msg1.header.stamp == new_msg1.header.stamp || old_msg2.header.stamp == new_msg2.header.stamp){
                continue;
            }


            //Calculate displacement and time difference
            geometry_msgs::PoseStamped scaled_output1 =  calcSpeedScale(old_msg1, new_msg1, actual_target1);
            geometry_msgs::PoseStamped scaled_output2 =  calcSpeedScale(old_msg2, new_msg2, actual_target2);



            //Check if any values are NAN
            if(std::isnan(scaled_output1.pose.position.x) || std::isnan(scaled_output1.pose.position.y) || std::isnan(scaled_output1.pose.position.z)){
                std::cout << "New message for 1 is nan" << std::endl;
                std::cout << scaled_output1.pose.position  << std::endl;
                return 0;
            }

            if(std::isnan(scaled_output2.pose.position.x) || std::isnan(scaled_output2.pose.position.y) || std::isnan(scaled_output2.pose.position.z)){
                std::cout << "New message for 2 is nan" << std::endl;
                std::cout << scaled_output2.pose.position  << std::endl;
                return 0;
            }

            //Set master position with newly scaled values!
            psm1.set_master_cart_pos(scaled_output1);
            psm2.set_master_cart_pos(scaled_output2);

            old_msg1 = new_msg1;
            old_msg2 = new_msg2;

            actual_target1 = scaled_output1;
            actual_target2 = scaled_output2;


            //passthrough for the jaw angle on master side
            psm1.set_master_jaw(psm1.get_target_master_jaw());
            psm2.set_master_jaw(psm2.get_target_master_jaw());

            //Passthrough for the slave side
            psm1.set_slave_jaw(psm1.get_target_slave_jaw());
            psm2.set_slave_jaw(psm2.get_target_slave_jaw());


            psm1.set_slave_cart_pos(psm1.get_target_slave_cart_pos());
            psm2.set_slave_cart_pos(psm2.get_target_slave_cart_pos());
        }

    }
}


geometry_msgs::PoseStamped calcSpeedScale(geometry_msgs::PoseStamped old_master, geometry_msgs::PoseStamped new_master, geometry_msgs::PoseStamped actual)
{
    //std:: cout << "made it to the function" << std::endl;

    //Set up position variables from message
    //*1 = old pose *2 = new pose
    float dx = new_master.pose.position.x - old_master.pose.position.x;
    float dy = new_master.pose.position.y - old_master.pose.position.y;
    float dz = new_master.pose.position.z - old_master.pose.position.z;

    float dt = (new_master.header.stamp - old_master.header.stamp).toSec();

    //Velocity in x,y,z
    float vx = dx/dt;
    float vy = dy/dt;
    float vz = dz/dt;

    //std::cout << "velocity: " << vx << " " << vy << " " << vz << std::endl;

    //"acceleration" in x,y,z
    float ax = (dx < 0.0) ? -vx * vx : vx * vx;
    float ay = (dy < 0.0) ? -vy * vy : vy * vy;
    float az = (dz < 0.0) ? -vz * vz : vz * vz;


//    std::cout << "ax :" << WEIGHT2 * ax << std::endl;
//    std::cout << "ay :" << WEIGHT2 * ay << std::endl;
//    std::cout << "az :" << WEIGHT2 * az << std::endl;
    //std::cout << "dt: " << dt << std::endl;


    //Rewrite new_master with equation

    float dxn = (fabs(WEIGHT1 * vx + WEIGHT2 * ax) * dt < fabs(SCALE_CAP * dx)) ? (WEIGHT1 * vx + WEIGHT2 * ax) * dt : SCALE_CAP * dx;
    float dyn = (fabs(WEIGHT1 * vy + WEIGHT2 * ay) * dt < fabs(SCALE_CAP * dy)) ? (WEIGHT1 * vy + WEIGHT2 * ay) * dt : SCALE_CAP * dy;
    float dzn = (fabs(WEIGHT1 * vz + WEIGHT2 * az) * dt < fabs(SCALE_CAP * dz)) ? (WEIGHT1 * vz + WEIGHT2 * az) * dt : SCALE_CAP * dz;

//    if ((dxn == SCALE_CAP*dx || dyn == SCALE_CAP*dy || dzn == SCALE_CAP*dz) && (dx != 0.0 && dy != 0.0 && dz != 0.0))
//    {
//        std::cout << "Capped" << dxn << " " << dx << " " << dyn << " " << dy << " " << dzn << " " << dz << std::endl;
//    }

//    std::cout << "vel: " << (WEIGHT1 * vx) * dt << " acc: " << WEIGHT2 * ax * dt << "   normal dx: " << dx << std::endl;
    //std::cout << "algorithm: " << dxn << " cap: " <<  SCALE_CAP * dx << std::endl;

//    dxn = 0;
//    dyn = 0;
//    dzn = 0;

   // std::cout << new_master.pose.position.x << " unchanged " ;

    actual.pose.position.x = dxn + actual.pose.position.x;
    actual.pose.position.y = dyn + actual.pose.position.y;
    actual.pose.position.z = dzn + actual.pose.position.z;


    //House keeping
    actual.header            = new_master.header;
    actual.pose.orientation  = new_master.pose.orientation;


    //std::cout << new_master.pose.position.x << " changed" << std::endl;

    return actual;
}
