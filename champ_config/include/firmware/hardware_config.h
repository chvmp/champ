#include <actuator.h>
#include <imu.h>

// (serial_interface, actuator_leg_id, actuator_driver_id,  min_angle,max_angle, inverted)
DynamixelAX12A::Plugin lfh_actuator(Serial1, 0, 16, 0, 0, false);
DynamixelAX12A::Plugin lfu_actuator(Serial1, 1, 17, 0, 0, false);
DynamixelAX12A::Plugin lfl_actuator(Serial1, 2, 18, 0, 0, true);

DynamixelAX12A::Plugin rfh_actuator(Serial1, 3, 14, 0, 0, false);
DynamixelAX12A::Plugin rfu_actuator(Serial1, 4, 13, 0, 0, true);
DynamixelAX12A::Plugin rfl_actuator(Serial1, 5, 4, 0, 0, false);

DynamixelAX12A::Plugin lhh_actuator(Serial1, 6, 10, 0, 0, false);
DynamixelAX12A::Plugin lhu_actuator(Serial1, 7, 11, 0, 0, false);
DynamixelAX12A::Plugin lhl_actuator(Serial1, 8, 12, 0, 0, true);

DynamixelAX12A::Plugin rhh_actuator(Serial1, 9, 6, 0, 0, false);
DynamixelAX12A::Plugin rhu_actuator(Serial1, 10, 5, 0, 0, true);
DynamixelAX12A::Plugin rhl_actuator(Serial1, 11, 8, 0, 0, false);

Actuator<DynamixelAX12A::Plugin> actuators
(
    lfh_actuator, lfu_actuator, lfl_actuator,
    rfh_actuator,rfu_actuator,rfl_actuator,
    lhh_actuator,lhu_actuator,lhl_actuator,
    rhh_actuator,rhu_actuator, rhl_actuator
);

IMU<BNO0809DOF::Plugin> imu;