#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <champ_msgs/Point.h>
#include <champ_msgs/PointArray.h>
#include <champ_msgs/Joints.h>
#include <champ_msgs/Pose.h>
#include <champ_msgs/Imu.h>
#include <champ_msgs/Velocities.h>
#include <quadruped_balancer.h>
#include <quadruped_gait.h>
#include <quadruped_ik.h>
#include <quadruped_description.h>
#include <gait_config.h>
#include <actuator_plugins.h>
#include <imu_plugins.h>
#include <hardware_config.h>
#include <odometry.h>

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

float g_req_roll = 0;
float g_req_pitch = 0;
float g_req_yaw = 0;

unsigned long g_prev_command_time = 0;

void velocityCommandCallback(const geometry_msgs::Twist& vel_cmd_msg);
void poseCommandCallback(const champ_msgs::Pose& pose_cmd_msg);

ros::NodeHandle nh;
champ_msgs::PointArray point_msg;
ros::Publisher point_pub("/champ/foot/raw", &point_msg);

champ_msgs::Joints joints_msg;
ros::Publisher jointstates_pub("/champ/joint_states/raw", &joints_msg);

champ_msgs::Imu imu_msg;
ros::Publisher imu_pub("/champ/imu/raw", &imu_msg);

champ_msgs::Velocities vel_msg;
ros::Publisher vel_pub("/champ/velocities/raw", &vel_msg);

ros::Subscriber<geometry_msgs::Twist> vel_cmd_sub("champ/cmd_vel", velocityCommandCallback);
ros::Subscriber<champ_msgs::Pose> pose_cmd_sub("champ/cmd_pose", poseCommandCallback);

QuadrupedBase base(lf_leg, rf_leg, lh_leg, rh_leg, KNEE_ORIENTATION);
QuadrupedBalancer balancer(base);
QuadrupedGait gait(base, MAX_LINEAR_VELOCITY_X, MAX_LINEAR_VELOCITY_Y, MAX_ANGULAR_VELOCITY_Z, 
                         STANCE_DURATION, SWING_HEIGHT, STANCE_DEPTH);
QuadrupedIK ik(base);
Odometry odometry(base);

void setup()
{
    joints_msg.position_length = 12;

    nh.initNode();
    nh.getHardware()->setBaud(500000);
    
    nh.advertise(point_pub);
    nh.advertise(jointstates_pub);
    nh.advertise(imu_pub);
    nh.advertise(vel_pub);

    nh.subscribe(vel_cmd_sub);
    nh.subscribe(pose_cmd_sub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    nh.loginfo("CHAMP CONNECTED");
    delay(1);
}

void loop() { 
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;

    static Accelerometer accel;
    static Gyroscope gyro;
    static Magnetometer mag;
    static Orientation rotation;

    if ((micros() - prev_control_time) >= 10000)
    {
        Transformation target_foot_positions[4];
        Transformation current_foot_positions[4];
        float target_joint_position[12]; 
        float current_joint_positions[12];
        Velocities velocities;
        
        actuators.getJointPositions(current_joint_positions);
        odometry.getVelocities(velocities);

        base.updateJointPositions(current_joint_positions);
        base.updateSpeed(velocities);

        balancer.setBodyPose(target_foot_positions, g_req_roll, g_req_pitch, g_req_yaw, NOMINAL_HEIGHT);
        gait.generate(target_foot_positions, g_req_linear_vel_x,  g_req_linear_vel_y, g_req_angular_vel_z);
        ik.solve(target_foot_positions, target_joint_position);
        
        actuators.moveJoints(target_joint_position);
        base.getFootPositions(current_foot_positions);

        publishPoints(current_foot_positions);
        publishJointStates(current_joint_positions);
        publishVelocities(velocities);

        prev_control_time = micros();
    }

    if ((micros() - prev_imu_time) >= 50000)
    {
        imu.read(rotation, accel, gyro, mag);
        publishIMU(rotation, accel, gyro, mag);

        prev_imu_time = micros();
    }

    if ((micros() - g_prev_command_time) >= 500000)
    {
        stopBase();
    }

    nh.spinOnce();
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void velocityCommandCallback(const geometry_msgs::Twist& vel_cmd_msg)
{
    g_req_linear_vel_x = vel_cmd_msg.linear.x;
    g_req_linear_vel_y = vel_cmd_msg.linear.y;
    g_req_angular_vel_z = vel_cmd_msg.angular.z;

    g_prev_command_time = micros();
}

void poseCommandCallback(const champ_msgs::Pose& pose_cmd_msg)
{
    g_req_roll = pose_cmd_msg.roll;
    g_req_pitch = pose_cmd_msg.pitch;
    g_req_yaw = pose_cmd_msg.yaw;
}

void publishJointStates(float joint_positions[12])
{
    joints_msg.position = joint_positions;
    jointstates_pub.publish(&joints_msg);  
}

void publishPoints(Transformation foot_positions[4])
{
    point_msg.lf.x = foot_positions[0].X();
    point_msg.lf.y = foot_positions[0].Y();
    point_msg.lf.z = foot_positions[0].Z();

    point_msg.rf.x = foot_positions[1].X();
    point_msg.rf.y = foot_positions[1].Y();
    point_msg.rf.z = foot_positions[1].Z();

    point_msg.lh.x = foot_positions[2].X();
    point_msg.lh.y = foot_positions[2].Y();
    point_msg.lh.z = foot_positions[2].Z();

    point_msg.rh.x = foot_positions[3].X();
    point_msg.rh.y = foot_positions[3].Y();
    point_msg.rh.z = foot_positions[3].Z();

    point_pub.publish(&point_msg);
}

void publishIMU(Orientation &rotation, Accelerometer &accel, Gyroscope &gyro, Magnetometer &mag)
{
    imu_msg.orientation.w = rotation.w;
    imu_msg.orientation.x = rotation.x;
    imu_msg.orientation.y = rotation.y;
    imu_msg.orientation.z = rotation.z;

    imu_msg.linear_acceleration.x = accel.x;
    imu_msg.linear_acceleration.y = accel.y;
    imu_msg.linear_acceleration.z = accel.z;

    imu_msg.angular_velocity.x = gyro.x;
    imu_msg.angular_velocity.y = gyro.y;
    imu_msg.angular_velocity.z = gyro.z;

    imu_msg.magnetic_field.x = mag.x;
    imu_msg.magnetic_field.y = mag.y;
    imu_msg.magnetic_field.z = mag.z;

    imu_pub.publish(&imu_msg);
}

void publishVelocities(Velocities vel)
{
    vel_msg.linear_x = vel.linear_velocity_x;
    vel_msg.linear_y = vel.linear_velocity_y;
    vel_msg.angular_z = vel.angular_velocity_z;

    //publish raw_vel_msg
    vel_pub.publish(&vel_msg);
}