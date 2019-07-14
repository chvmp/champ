#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <champ_msgs/Point.h>
#include <champ_msgs/PointArray.h>
#include <champ_msgs/Joints.h>
#include <champ_description.h>
#include <champ_config.h>
#include <quadruped_ik.h>
#include <quadruped_balancer.h>
#include <quadruped_gait.h>

#define CONTROL_RATE FREQUENCY

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

void commandCallback(const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle nh;
champ_msgs::PointArray point_msg;
ros::Publisher point_pub("/champ/foot/raw", &point_msg);

champ_msgs::Joints joints_msg;
ros::Publisher jointstates_pub("/champ/joint_states/raw", &joints_msg);

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

QuadrupedBase base(lf_leg, rf_leg, lh_leg, rh_leg);
QuadrupedBalancer balancer(base);
QuadrupedIK ik(base);
QuadrupedGait gait(base, MAX_VELOCITY, SWING_HEIGHT, STEP_LENGTH, STANCE_DEPTH);

void setup()
{
    joints_msg.position_length = 12;

    nh.initNode();
    nh.getHardware()->setBaud(115200);
    nh.advertise(point_pub);
    nh.advertise(jointstates_pub);
    nh.subscribe(cmd_sub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    nh.loginfo("CHAMP CONNECTED");
    delay(1);
}

void loop() { 
    static unsigned long prev_control_time = 0;

    if ((millis() - prev_control_time) >= (1000 / CONTROL_RATE))
    {
        base.lf->joints(0, 0, 0);
        base.rf->joints(0, 0, 0);
        base.lh->joints(0, 0, 0);
        base.rh->joints(0, 0, 0);
        base.attitude(0.0, 0.0, 0.0);

        balancer.balance(0.0, 0.0, 0.0, 0.0, 0.0, -0.08);
        gait.generate(balancer.lf.stance(), balancer.rf.stance(), balancer.lh.stance(), balancer.rh.stance(), g_req_linear_vel_x,  g_req_linear_vel_y, g_req_angular_vel_z);
        ik.solve(gait.lf.stance(), gait.rf.stance(), gait.lh.stance(), gait.rh.stance());

        publishPoints(gait.lf.stance(), gait.rf.stance(), gait.lh.stance(), gait.rh.stance());
        publishJointStates(ik.lf.joints(), ik.rf.joints(), ik.lh.joints(), ik.rh.joints());

        prev_control_time = millis();
    }

    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }

    nh.spinOnce();
}

void publishJointStates(float lf_joints[3], float rf_joints[3], float lh_joints[3], float rh_joints[3])
{
    float target_joints[12];
    target_joints[0] = lf_joints[0];
    target_joints[1] = lf_joints[1];
    target_joints[2] = lf_joints[2];

    target_joints[3] = rf_joints[0];
    target_joints[4] = rf_joints[1];
    target_joints[5] = rf_joints[2];

    target_joints[6] = lh_joints[0];
    target_joints[7] = lh_joints[1];
    target_joints[8] = lh_joints[2];

    target_joints[9] = rh_joints[0];
    target_joints[10] = rh_joints[1];
    target_joints[11] = rh_joints[2];

    joints_msg.position = target_joints;
    jointstates_pub.publish(&joints_msg);
}

void publishPoints(Transformation p_lf, Transformation p_rf, Transformation p_lh, Transformation p_rh)
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

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
}