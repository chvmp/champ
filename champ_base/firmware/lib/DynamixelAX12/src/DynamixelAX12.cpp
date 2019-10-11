#include "DynamixelAX12.h"


DynamixelAX12::DynamixelAX12(OneWireMInterface &aInterface, uint8_t aId) :
    DynamixelDevice(aInterface, aId)
{}

OneWireStatus DynamixelAX12::wheelMode()
{
    return jointMode(0, 0);
}

OneWireStatus DynamixelAX12::jointMode(uint16_t aCWLimit, uint16_t aCCWLimit)
{
    uint32_t data = (aCWLimit | (uint32_t(aCCWLimit) << 16));
    return write(DYN_ADDRESS_CW_LIMIT, data);
}

OneWireStatus DynamixelAX12::enableTorque(bool aTorque)
{
    return write(DYN_ADDRESS_ENABLE_TORQUE, uint8_t(aTorque ? 1 : 0));
}

OneWireStatus DynamixelAX12::alarmShutdown(uint8_t aMode)
{
    aMode &= B01111111;
    return write(DYN_ADDRESS_ALARM_SHUTDOWN, aMode);
}

OneWireStatus DynamixelAX12::speed(uint16_t aSpeed)
{
    return write(DYN_ADDRESS_GOAL_SPEED, aSpeed);
}

OneWireStatus DynamixelAX12::torqueLimit(uint16_t aTorque)
{
    return write(DYN_ADDRESS_TORQUE_LIMIT, aTorque);
}

OneWireStatus DynamixelAX12::goalPosition(uint16_t aPosition)
{
    return write(DYN_ADDRESS_GOAL_POSITION, aPosition);
}

OneWireStatus DynamixelAX12::goalPositionDegree(uint16_t posDeg)
{
    return goalPosition(posDeg * 3.41);
}

OneWireStatus DynamixelAX12::recoverTorque()
{
    OneWireStatus ret;
    uint16_t currentPosition = UINT16_MAX;
    uint16_t torque = 0x3FF;

    // Cannot recover AX12 if we cannot read registers
    if (statusReturnLevel() == 0)
    {
        return OW_STATUS_COM_ERROR;
    }

    ret = read(DYN_ADDRESS_CURRENT_POSITION, currentPosition);
    if (ret != OW_STATUS_OK)
    {
        return ret;
    }
    if (currentPosition == UINT16_MAX)
    {
        return OW_STATUS_COM_ERROR;
    }
    ret = write(DYN_ADDRESS_GOAL_POSITION, currentPosition);
    if (ret != OW_STATUS_OK)
    {
        return ret;
    }
    ret = read(DYN_ADDRESS_MAX_TORQUE, torque);
    if (ret != OW_STATUS_OK)
    {
        ret = write(DYN_ADDRESS_TORQUE_LIMIT, 0x3FF);
    }
    else
    {
        ret = write(DYN_ADDRESS_TORQUE_LIMIT, torque);
    }
    if (ret != OW_STATUS_OK)
    {
        return ret;
    }
    return enableTorque();
}

OneWireStatus DynamixelAX12::resetSecuritySettings()
{
    OneWireStatus status = OW_STATUS_OK;
    status |= write(DYN_ADDRESS_TEMP_LIMIT, (uint8_t)85);
    status |= write(DYN_ADDRESS_LOW_VOLTAGE_LIMIT, (uint8_t)60);
    status |= write(DYN_ADDRESS_HIGH_VOLTAGE_LIMIT, (uint8_t)140);
    status |= write(DYN_ADDRESS_MAX_TORQUE, (uint16_t)0x3FF);
    status |= write(DYN_ADDRESS_ALARM_LED, (uint8_t)36);
    status |= write(DYN_ADDRESS_ALARM_SHUTDOWN, (uint8_t)36);
    return status;
}

OneWireStatus DynamixelAX12::led(uint8_t aState)
{
    return write(DYN_ADDRESS_LED, aState);
}

OneWireStatus DynamixelAX12::currentPosition(uint16_t &aPosition)
{
    aPosition = UINT16_MAX;
    return read(DYN_ADDRESS_CURRENT_POSITION, aPosition);
}

OneWireStatus DynamixelAX12::currentPositionDegree(uint16_t &aPosition)
{
    OneWireStatus status = currentPosition(aPosition);
    if (aPosition != UINT16_MAX)
    {
        aPosition = (uint16_t)(0.5 + (float)aPosition / 3.41);
    }
    return status;
}
