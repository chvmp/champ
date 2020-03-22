
#include <hardware_config.h>
#include <constructors/constructors.h>

#include <quadruped_description.h>
#include <gait_config.h>

#include <body_controller/body_controller.h>
#include <leg_controller/leg_controller.h>
#include <ik_engine/ik_engine.h>
#include <odometry/odometry.h>

champ::QuadrupedBase base(lf_leg, rf_leg, lh_leg, rh_leg, gait_config);
champ::BodyController body_controller(base);
champ::LegController leg_controller(base);
champ::IKEngine ik(base);
champ::Odometry odometry(base);

void setup()
{
}

void loop() { 
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_publish_time = 0;

    geometry::Transformation target_foot_positions[4];
    float target_joint_positions[12]; 
    
    if ((micros() - prev_control_time) >= 2000)
    {
        prev_control_time = micros();

        champ::Pose req_pose;
        command_interface.poseInput(req_pose);
        body_controller.poseCommand(target_foot_positions, req_pose);

        champ::Velocities req_vel;
        command_interface.velocityInput(req_vel);
        leg_controller.velocityCommand(target_foot_positions, req_vel);

        ik.solve(target_joint_positions, target_foot_positions);

        command_interface.jointsInput(target_joint_positions);
        actuators.moveJoints(target_joint_positions);
    }

    if ((micros() - prev_publish_time) >= 20000)
    {
        prev_publish_time = micros();

        geometry::Transformation current_foot_positions[4];
        float current_joint_positions[12];
        champ::Velocities current_speed;

        actuators.getJointPositions(current_joint_positions);
        base.updateJointPositions(current_joint_positions);
        base.getFootPositions(current_foot_positions);

        odometry.getVelocities(current_speed);
        status_interface.publishVelocities(current_speed);
        status_interface.publishPoints(target_foot_positions);
        status_interface.publishJointStates(current_joint_positions);
    }

    if ((micros() - prev_imu_time) >= 50000)
    {
        prev_imu_time = micros();

        champ::Accelerometer accel;
        champ::Gyroscope gyro;
        champ::Magnetometer mag;
        champ::Quaternion orientation;

        imu.read(orientation, accel, gyro, mag);
        status_interface.publishIMU(orientation, accel, gyro, mag);
    }

    command_interface.run();
    imu.run();
}