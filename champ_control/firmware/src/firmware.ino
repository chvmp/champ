#include <ros.h>
#include <ros/time.h>
#include <champ_msgs/Point.h>
#include <champ_msgs/Joints.h>

#include <Geometry.h>
#include <Geometry.h>
#include <DH.h>
#include <StateTracker.h>
#define CONTROL_RATE 100

LegJoints leg_states;
LegJoints target_states;

ros::NodeHandle nh;
champ_msgs::Point point_msg;
ros::Publisher point_pub("point_debug", &point_msg);

champ_msgs::Joints joints_msg;
ros::Publisher jointstates_pub("champ/joint_states", &joints_msg);

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
        leg_states.lf.joints(1,2,3);
        leg_states.rf.joints(4,5,6);
        leg_states.lh.joints(7,8,9);
        leg_states.rh.joints(10,11,12);
        publishJointStates(leg_states.joints());


        Transformation target;
        // float target_joints[3];

        // float zeros[3] = {0.61, 0.71, 0.0};
        // rf_leg.updateJoints(zeros);
        // target = rf_leg.forwardKinematics();
        //TODO: THIS FUNCTION IS NOT WORKING PROPERLY
        //THIS IS RETURNING NANs
        rf_leg.inverseKinematics(target, target_states.lf.joints_);

         char buffer[50];
        sprintf (buffer, "Joint0  : %f", target_states.lf.hip());
        nh.loginfo(buffer);

        sprintf (buffer, "Joint1  : %f", target_states.lf.upper_leg());
        nh.loginfo(buffer);

        sprintf (buffer, "Joint2  : %f", target_states.lf.lower_leg());
        nh.loginfo(buffer);

        // point_msg.x = target.X();
        // point_msg.y = target.Y(); 
        // point_msg.z = target.Z();
        // point_pub.publish(&point_msg);
        prev_ik_time = millis();
    }
    nh.spinOnce();
}

void publishJointStates(float *joints)
{
    joints_msg.position = joints;
    jointstates_pub.publish(&joints_msg);
}