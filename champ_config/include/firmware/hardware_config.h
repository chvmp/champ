#include <actuator.h>
#include <imu.h>

#define USE_SIMULATION_ACTUATOR
// #define USE_DYNAMIXEL_ACTUATOR
// #define USE_SERVO_ACTUATOR
// #define USE_BRUSHLESS_ACTUATOR

#define USE_SIMULATION_IMU
// #define USE_BNO0809DOF_IMU

#ifdef USE_SIMULATION_ACTUATOR
    #define ACTUATOR SimulationActuator
    SimulationActuator::Plugin lfh_actuator;
    SimulationActuator::Plugin lfu_actuator;
    SimulationActuator::Plugin lfl_actuator;

    SimulationActuator::Plugin rfh_actuator;
    SimulationActuator::Plugin rfu_actuator;
    SimulationActuator::Plugin rfl_actuator;

    SimulationActuator::Plugin lhh_actuator;
    SimulationActuator::Plugin lhu_actuator;
    SimulationActuator::Plugin lhl_actuator;

    SimulationActuator::Plugin rhh_actuator;
    SimulationActuator::Plugin rhu_actuator;
    SimulationActuator::Plugin rhl_actuator;
#endif 

Actuator<ACTUATOR::Plugin> actuators
(
    PANTOGRAPH_LEG,
    lfh_actuator, lfu_actuator, lfl_actuator,
    rfh_actuator,rfu_actuator,rfl_actuator,
    lhh_actuator,lhu_actuator,lhl_actuator,
    rhh_actuator,rhu_actuator, rhl_actuator
);

#ifdef USE_SIMULATION_IMU
    IMU<SimulationIMU::Plugin> imu;
#endif

#ifdef USE_BNO0809DOF_IMU
    IMU<BNO0809DOF::Plugin> imu;
#endif