#include <ros.h>
#include <ros/time.h>
#include <champ_msgs/Point.h>
#include <Geometry.h>
#include <Geometry.h>
#include <DH.h>

#define CONTROL_RATE 150

ros::NodeHandle nh;
champ_msgs::Point point_msg;
ros::Publisher point_pub("point_debug", &point_msg);

void setup()
{
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.advertise(point_pub);

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
        Transformation target;
        float target_joints[3];

        float zeros[3] = {0.61, 0.71, 0.0};
        rf_leg.updateJoints(zeros);
        target = rf_leg.forwardKinematics();

        rf_leg.inverseKinematics(target, target_joints);

        char buffer[50];
        sprintf (buffer, "Joint0  : %f", target_joints[0]);
        nh.loginfo(buffer);

        sprintf (buffer, "Joint1  : %f", target_joints[1]);
        nh.loginfo(buffer);

        sprintf (buffer, "Joint2  : %f", target_joints[2]);
        nh.loginfo(buffer);

        point_msg.x = target.X();
        point_msg.y = target.Y(); 
        point_msg.z = target.Z();
        point_pub.publish(&point_msg);
        prev_ik_time = millis();
    }
    nh.spinOnce();
}
