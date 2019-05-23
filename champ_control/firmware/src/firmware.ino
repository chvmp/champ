#include <ros.h>
#include <ros/time.h>
#include <champ_msgs/Point.h>
#include <champ_msgs/PointArray.h>
#include <champ_msgs/Joints.h>
#include <champ_description.h>
#include <quadruped_ik.h>
#include <quadruped_balancer.h>

#define CONTROL_RATE 100

ros::NodeHandle nh;
champ_msgs::PointArray point_msg;
ros::Publisher point_pub("/champ/ee/raw", &point_msg);

champ_msgs::Joints joints_msg;
ros::Publisher jointstates_pub("/champ/joint_states/raw", &joints_msg);

QuadrupedBase base(lf_leg, rf_leg, lh_leg, rh_leg);
QuadrupedBalancer balancer(base);
QuadrupedIK ik;

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
        float target_joint_states[12];
        float current_joint_states[12];

        // //update joint states of the robot
        // base.lf->joints(0, 0.89, -1.75);
        // base.rf->joints(0, 0.89, -1.75);
        // base.lh->joints(0, 0.89, -1.75);
        // base.rh->joints(0, 0.89, -1.75);
        base.lf->joints(0, 0, 0);
        base.rf->joints(0, 0, 0);
        base.lh->joints(0, 0, 0);
        base.rh->joints(0, 0, 0);
        base.attitude(0.0, 0.0, 0.0);

        // Point target;
        // target.X() = 0.187;
        // target.Y() = -0.012;
        // target.Z() = -0.138;
        // ik.solveBody(base, target, target, target, target, target_joint_states);
        // publishJointStates(target_joint_states);
        
        balancer.balance(0.3, 0.3, 0.0, 0.0, 0.0, -0.150 );
        publishPoints(balancer.lf_stance().p, balancer.rf_stance().p, balancer.lh_stance().p, balancer.rh_stance().p);

        ik.solveBody(base, balancer.lf_stance().p, balancer.rf_stance().p, balancer.lh_stance().p, balancer.rh_stance().p, target_joint_states);
        publishJointStates(target_joint_states);

        //publish all joint angles
        // base.joints(current_joint_states);
        // publishJointStates(current_joint_states);

        //publish ee points
        // publishPoints(base.lf->ee().p, base.rf->ee().p, base.lh->ee().p, base.rh->ee().p);
        // publishPoints(base.lf->ee_to_base().p, base.rf->ee_to_base().p, base.lh->ee_to_base().p, base.rh->ee_to_base().p);

        prev_ik_time = millis();
    }
    nh.spinOnce();
}

void publishJointStates(float *joints)
{
    joints_msg.position = joints;
    jointstates_pub.publish(&joints_msg);
}

void publishPoints(Point p_lf, Point p_rf, Point p_lh, Point p_rh)
{
    point_msg.lf.x = p_lf.X();
    point_msg.lf.y = p_lf.Y();
    point_msg.lf.z = p_lf.Z();

    point_msg.rf.x = p_rf.X();
    point_msg.rf.y = p_rf.Y();
    point_msg.rf.z = p_rf.Z();

    point_msg.lh.x = p_lh.X();
    point_msg.lh.y = p_lh.Y();
    point_msg.lh.z = p_lh.Z();

    point_msg.rh.x = p_rh.X();
    point_msg.rh.y = p_rh.Y();
    point_msg.rh.z = p_rh.Z();

    point_pub.publish(&point_msg);
}