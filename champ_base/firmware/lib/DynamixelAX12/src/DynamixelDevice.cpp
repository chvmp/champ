#include "DynamixelDevice.h"


DynamixelDevice::DynamixelDevice(OneWireMInterface &aInterface, uint8_t aID) :
	mInterface(aInterface), mStatusReturnLevel(0), mID(aID)
{
	mStatus = DYN_STATUS_OK;
}

OneWireStatus DynamixelDevice::init()
{
    mStatusReturnLevel = 1;
    OneWireStatus ret = ping();
    if (ret != OW_STATUS_OK)
    {
        mStatusReturnLevel = 0;
        return ret;
    }
    ret = read(DYN_ADDRESS_SRL, mStatusReturnLevel);
    if (ret != OW_STATUS_OK)
    {
        mStatusReturnLevel = 0;
    }
    if (ret == OW_STATUS_TIMEOUT)
    {
        return OW_STATUS_OK;
    }
    else
    {
        return ret;
    }
}

bool DynamixelDevice::environmentError() const
{
    return (mStatus & (
        DYN_STATUS_INPUT_VOLTAGE_ERROR |
        DYN_STATUS_OVERHEATING_ERROR |
        DYN_STATUS_OVERLOAD_ERROR)) != 0;
}

bool DynamixelDevice::commandError() const
{
    return (mStatus & (
        DYN_STATUS_ANGLE_LIMIT_ERROR |
        DYN_STATUS_RANGE_ERROR |
        DYN_STATUS_CHECKSUM_ERROR |
        DYN_STATUS_INSTRUCTION_ERROR)) != 0;
}

bool DynamixelDevice::recoverableError() const
{
    return (mStatus & DYN_STATUS_OVERLOAD_ERROR) != 0 &&
        (mStatus & (DYN_STATUS_INPUT_VOLTAGE_ERROR | 
        DYN_STATUS_OVERHEATING_ERROR)) == 0;
}

int DynamixelDevice::setId(uint8_t newId)
{
    OneWireStatus ret = write(DYN_ADDRESS_ID, newId);
    if (ret == OW_STATUS_OK && !commandError())
    {
        mID = newId;
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}

uint16_t DynamixelDevice::model()
{
    uint16_t result = 0;
    if (read(DYN_ADDRESS_MODEL, result) == OW_STATUS_OK)
    {
        return result;
    }
    else
    {
        return 0;
    }
}

uint8_t DynamixelDevice::firmware()
{
    uint8_t result = 0;
    if (read(DYN_ADDRESS_FIRMWARE, result) == OW_STATUS_OK)
    {
        return result;
    }
    else
    {
        return 0;
    }
}

int DynamixelDevice::communicationSpeed(uint32_t aBaudrate)
{
    uint8_t value = 2000000 / aBaudrate - 1;
    if (value == 0) // forbid 2MBd rate, because it is out of spec, and can be difficult to undo
    {
        return EXIT_FAILURE;
    }
    OneWireStatus ret = write(DYN_ADDRESS_BAUDRATE, value);
    if (ret == OW_STATUS_OK && !commandError())
    {
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}

int DynamixelDevice::statusReturnLevel(uint8_t aSRL)
{
    OneWireStatus ret = write(DYN_ADDRESS_SRL, aSRL);
	if (ret == OW_STATUS_OK && !commandError())
	{
		mStatusReturnLevel = aSRL;
        return EXIT_SUCCESS;
	}
    else
    {
        return EXIT_FAILURE;
    }
}
