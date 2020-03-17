
#include <quadruped_description.h>
#include <gait_config.h>
#include <hardware_config.h>
#include <constructors/constructors.h>

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

    Transformation target_foot_positions[4];
    float target_joint_positions[12]; 
    
    if ((micros() - prev_control_time) >= 2000)
    {
        prev_control_time = micros();

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

        Transformation current_foot_positions[4];
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
        champ::Orientation rotation;

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
