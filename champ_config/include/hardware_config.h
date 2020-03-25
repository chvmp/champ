#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#define USE_SIMULATION_ACTUATOR
// #define USE_DYNAMIXEL_ACTUATOR
// #define USE_SERVO_ACTUATOR
// #define USE_BRUSHLESS_ACTUATOR

#define USE_SIMULATION_IMU
// #define USE_BNO0809DOF_IMU

#define USE_ROS
// #define USE_ROS_RF

#ifdef USE_ROS_RF
    #define ELE_PIN 16
    #define AIL_PIN 21
    #define RUD_PIN 17
    #define THR_PIN 20
    #define AUX1_PIN 22
    #define AUX2_PIN 23
    #define RF_INV_LX false
    #define RF_INV_LY false
    #define RF_INV_AZ false
    #define RF_INV_ROLL true
    #define RF_INV_PITCH false
    #define RF_INV_YAW false
#endif

#ifdef USE_DYNAMIXEL_ACTUATOR
    #define LFH_SERVO_ID 16
    #define LFU_SERVO_ID 17
    #define LFL_SERVO_ID 18

    #define RFH_SERVO_ID 14
    #define RFU_SERVO_ID 7
    #define RFL_SERVO_ID 4

    #define LHH_SERVO_ID 2
    #define LHU_SERVO_ID 11
    #define LHL_SERVO_ID 12

    #define RHH_SERVO_ID 6
    #define RHU_SERVO_ID 5
    #define RHL_SERVO_ID 8

    #define LFH_INV false
    #define LFU_INV false
    #define LFL_INV true

    #define RFH_INV false
    #define RFU_INV true
    #define RFL_INV false

    #define LHH_INV false
    #define LHU_INV false
    #define LHL_INV true

    #define RHH_INV false
    #define RHU_INV true
    #define RHL_INV false
#endif 

#ifdef USE_SERVO_ACTUATOR
    #define LFH_PIN 2
    #define LFU_PIN 3
    #define LFL_PIN 4

    #define RFH_PIN 23
    #define RFU_PIN 22
    #define RFL_PIN 21

    #define LHH_PIN 6
    #define LHU_PIN 7
    #define LHL_PIN 8

    #define RHH_PIN 17
    #define RHU_PIN 16
    #define RHL_PIN 14

    #define LFH_INV false
    #define LFU_INV false
    #define LFL_INV true

    #define RFH_INV false
    #define RFU_INV true
    #define RFL_INV false

    #define LHH_INV false
    #define LHU_INV false
    #define LHL_INV true

    #define RHH_INV false
    #define RHU_INV true
    #define RHL_INV false
#endif

#endif