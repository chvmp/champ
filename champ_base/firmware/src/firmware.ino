/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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