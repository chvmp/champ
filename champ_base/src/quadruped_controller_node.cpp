#include "ros/ros.h"
#include "quadruped_controller.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "champ_controller_node");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    
    QuadrupedController champ(nh, nh_private);
    
    ros::spin();
    return 0;
}