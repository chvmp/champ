#include <ros.h>
#include <ros/time.h>

#include <quadruped_description.h>
#include <gait_config.h>
#include <body_controller/body_controller.h>
#include <leg_controller/leg_controller.h>
#include <ik_engine/ik_engine.h>
#include <actuator_plugins.h>
#include <imu_plugins.h>
#include <hardware_config.h>
#include <odometry/odometry.h>
#include <comms/input_interfaces/input_interface.h>
#include <comms/status_interfaces/status_interface.h>

champ::Interfaces::ROSSerial ros_interface;
champ::Interfaces::RF rf_interface(16, 21,17,20,22,23);
champ::Interfaces::Input<champ::Interfaces::ROSSerial, champ::Interfaces::RF> command_interface(ros_interface, rf_interface);
champ::Interfaces::Status<champ::Interfaces::ROSSerial> status_interface(ros_interface);

champ::QuadrupedBase base(lf_leg, rf_leg, lh_leg, rh_leg, KNEE_ORIENTATION, PANTOGRAPH_LEG);
champ::BodyController body_controller(base);
champ::LegController leg_controller(base, MAX_LINEAR_VELOCITY_X, MAX_LINEAR_VELOCITY_Y, MAX_ANGULAR_VELOCITY_Z, 
                         STANCE_DURATION, SWING_HEIGHT, STANCE_DEPTH);
champ::IKEngine ik(base);
champ::Odometry odometry(base);

void setup()
{
}

void loop() { 
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_publish_time = 0;

    static Transformation current_foot_positions[4];
    static float current_joint_positions[12];
    
    static champ::Accelerometer accel;
    static champ::Gyroscope gyro;
    static champ::Magnetometer mag;
    static champ::Orientation rotation;
  
    static champ::Velocities velocities;

    if ((micros() - prev_control_time) >= 2000)
    {
        prev_control_time = micros();

        Transformation target_foot_positions[4];
        float target_joint_positions[12]; 

        champ::Pose req_pose;
        command_interface.poseInput(req_pose);
        body_controller.poseCommand(target_foot_positions, 
            req_pose.roll, 
            req_pose.pitch, 
            req_pose.yaw, 
            NOMINAL_HEIGHT);

        champ::Velocities req_vel;
        command_interface.velocityInput(req_vel);
        leg_controller.velocityCommand(target_foot_positions, 
            req_vel.linear_velocity_x, 
            req_vel.linear_velocity_y, 
            req_vel.angular_velocity_z);

        ik.solve(target_joint_positions, target_foot_positions);

        command_interface.jointsInput(target_joint_positions);
        actuators.moveJoints(target_joint_positions);
    }

    if ((micros() - prev_publish_time) >= 20000)
    {
        prev_publish_time = micros();

        odometry.getVelocities(velocities);
        status_interface.publishVelocities(velocities);

        actuators.getJointPositions(current_joint_positions);
        base.updateJointPositions(current_joint_positions);
        base.getFootPositions(current_foot_positions);

        status_interface.publishPoints(current_foot_positions);
        status_interface.publishJointStates(current_joint_positions);
    }

    if ((micros() - prev_imu_time) >= 50000)
    {
        prev_imu_time = micros();

        imu.read(rotation, accel, gyro, mag);
        status_interface.publishIMU(rotation, accel, gyro, mag);
    }

    command_interface.run();
    imu.run();
}

void standUp()
{
    Transformation target_foot_positions[4];
    float target_joint_positions[12]; 
    float current_joint_positions[12];

    float initial_height = NOMINAL_HEIGHT * 0.75;
    float hip_angle = 0.785398;
    
    actuators.getJointPositions(current_joint_positions);
    base.updateJointPositions(current_joint_positions);

    float sum_of_hip_angles = current_joint_positions[0] + current_joint_positions[3] +
                              current_joint_positions[6] + current_joint_positions[9];

    if(abs(sum_of_hip_angles) > 0.349066)
    {
        body_controller.poseCommand(target_foot_positions, 0, 0, 0, initial_height);
        ik.solve(target_joint_positions, target_foot_positions);
        target_joint_positions[0] = hip_angle;
        target_joint_positions[3] = -hip_angle;
        target_joint_positions[6] = hip_angle;
        target_joint_positions[9] = -hip_angle;
        actuators.moveJoints(target_joint_positions);
        delay(1000);

        for(int i = 100; i > -1; i--)
        {
            float current_angle = (i / 100.0) * hip_angle;
            target_joint_positions[0] = current_angle;
            target_joint_positions[3] = -current_angle;
            target_joint_positions[6] = current_angle;
            target_joint_positions[9] = -current_angle;
            actuators.moveJoints(target_joint_positions);
            delay(5);
        }
        delay(500);
    }

    float average_leg_height = (base.legs[0]->foot_from_hip().Z() + base.legs[1]->foot_from_hip().Z() +
                                base.legs[2]->foot_from_hip().Z() + base.legs[3]->foot_from_hip().Z()) / 4.0;

    if(abs(NOMINAL_HEIGHT + average_leg_height) > 0.01)
    {
        float current_height = initial_height;
        for(unsigned int i = 0; i < 101; i++)
        {
            current_height += (NOMINAL_HEIGHT - initial_height) / 100.0;
            body_controller.poseCommand(target_foot_positions, 0, 0, 0, current_height);
            ik.solve(target_joint_positions, target_foot_positions);
            actuators.moveJoints(target_joint_positions);
            delay(2);
        }
    }
}
