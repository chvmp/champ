#ifndef DYNAMIXEL_AX12_H
#define DYNAMIXEL_AX12_H

#include "DynamixelDevice.h"

#ifndef UINT16_MAX
#define UINT16_MAX (65535)
#endif // !UINT16_MAX


class DynamixelAX12 : public DynamixelDevice
{
    public:
        DynamixelAX12(OneWireMInterface &aInterface, uint8_t aId);

        OneWireStatus wheelMode();
        OneWireStatus jointMode(uint16_t aCWLimit = 0, uint16_t aCCWLimit = 0x3FF);
        OneWireStatus enableTorque(bool aTorque = true);
        OneWireStatus recoverTorque();

        OneWireStatus alarmShutdown(uint8_t aMode = 0x04);
        OneWireStatus speed(uint16_t aSpeed);
        OneWireStatus torqueLimit(uint16_t aTorque);
        OneWireStatus goalPosition(uint16_t aPosition);
        OneWireStatus goalPositionDegree(uint16_t posDeg);
        OneWireStatus resetSecuritySettings();
        OneWireStatus led(uint8_t aState);
        OneWireStatus currentPosition(uint16_t &aPosition);
        OneWireStatus currentPositionDegree(uint16_t &aPosition);
        OneWireStatus changeRDT(uint8_t delay);

        void setActuatorID(uint8_t newId);
        void changeActuatorID(uint8_t newId);
};

#endif
