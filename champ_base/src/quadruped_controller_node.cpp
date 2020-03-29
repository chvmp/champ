#include "ros/ros.h"
#include "quadruped_controller.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "champ_controller_node");
    QuadrupedController champ;
    return 0;
}