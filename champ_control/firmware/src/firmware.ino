#include <ros.h>
#include <ros/time.h>
#include <champ_msgs/Point.h>
#include <champ_msgs/Joints.h>
#include <champ_description.h>
#define CONTROL_RATE 100

ros::NodeHandle nh;
champ_msgs::Point point_msg;
ros::Publisher point_pub("/champ/ee/raw", &point_msg);

champ_msgs::Joints joints_msg;
ros::Publisher jointstates_pub("/champ/joint_states/raw", &joints_msg);

void setup()
{
    joints_msg.position_length = 12;

    nh.initNode();
    nh.getHardware()->setBaud(115200);
    nh.advertise(point_pub);
    nh.advertise(jointstates_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    nh.loginfo("CHAMP CONNECTED");
    delay(1);
}

void loop() { 
    static unsigned long prev_ik_time = 0;

    if ((millis() - prev_ik_time) >= (1000 / CONTROL_RATE))
    {
        //update joint states of the robot
        b.lf->joints(0, 89, -1.75);
        b.rf->joints(0, 89, -1.75);
        b.lh->joints(0, 89, -1.75);
        b.rh->joints(0, 89, -1.75);

        //publish all joint angles
        publishJointStates();
        prev_ik_time = millis();
    }
    nh.spinOnce();
}

void publishJointStates()
{
    float joints[12];
    unsigned int total_joints = 0;

    for(int i = 0; i < 4; i++)
    {
        joints[total_joints++] = b.legs[i]->hip();
        joints[total_joints++] = b.legs[i]->upper_leg();
        joints[total_joints++] = b.legs[i]->lower_leg();
    }

    joints_msg.position = joints;
    jointstates_pub.publish(&joints_msg);
}