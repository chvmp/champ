#include <ros.h>
#include <ros/time.h>
#include <champ_msgs/Point.h>
#include <champ_msgs/Joints.h>
#include <champ_description.h>
#include <quadruped_ik.h>

#define CONTROL_RATE 100

QuadrupedIK ik;

ros::NodeHandle nh;
champ_msgs::Point point_msg;
ros::Publisher point_pub("/champ/ee/raw", &point_msg);

champ_msgs::Joints joints_msg;
ros::Publisher jointstates_pub("/champ/joint_states/raw", &joints_msg);

void setup()
{

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
        float target_joint_states[12];
        float current_joint_states[12];

        // //update joint states of the robot
        b.lf->joints(0, 0.89, -1.75);
        b.rf->joints(0, 0.89, -1.75);
        b.lh->joints(0, 0.89, -1.75);
        b.rh->joints(0, 0.89, -1.75);

        // Transformation target;
        // target.X() = 0.187;
        // target.Y() = -0.012;
        // target.Z() = -0.138;
        // ik.solveBody(b, target, target, target, target, target_joint_states);
        // publishJointStates(target_joint_states);
        
        //publish all joint angles
        b.joints(current_joint_states);
        publishJointStates(current_joint_states);

        //publish ee points
        publishPoints();

        prev_ik_time = millis();
    }
    nh.spinOnce();
}

void publishJointStates(float *joints)
{
    joints_msg.position_length = 12;
    joints_msg.position = joints;
    jointstates_pub.publish(&joints_msg);
}

void publishPoints()
{
    point_msg.x = b.lf->ee().p.X();
    point_msg.y = b.lf->ee().p.Y();
    point_msg.z = b.lf->ee().p.Z();

    point_pub.publish(&point_msg);
}