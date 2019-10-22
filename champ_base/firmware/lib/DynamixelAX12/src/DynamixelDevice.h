#ifndef DYNAMIXEL_DEVICE_H
#define DYNAMIXEL_DEVICE_H

#include "Dynamixel.h"
#include "OneWireMInterface.h"


class DynamixelDevice
{
public:
	DynamixelDevice(OneWireMInterface &aInterface, uint8_t aId);
	
	OneWireStatus init();
	
	DynamixelStatus status() const { return mStatus; }
    bool error() const { return mStatus != DYN_STATUS_OK; }
    bool environmentError() const;
    bool commandError() const;
    bool recoverableError() const;
	
	uint8_t id() const { return mID; }
    int setId(uint8_t newId);	
	uint16_t model();
	uint8_t firmware();
    int communicationSpeed(uint32_t aBaudrate);
    int statusReturnLevel(uint8_t aSRL);
    uint8_t statusReturnLevel() const { return mStatusReturnLevel; }
	
	template<class T>
	inline OneWireStatus read(uint8_t aAddress, T& aData)
	{
		return mInterface.read<T>(mID, aAddress, aData, mStatusReturnLevel, &mStatus);
	}
	
	template<class T>
	inline OneWireStatus write(uint8_t aAddress, const T& aData)
	{
		return mInterface.write<T>(mID, aAddress, aData, mStatusReturnLevel, &mStatus);
	}
	
	template<class T>
	inline OneWireStatus regWrite(uint8_t aAddress, const T& aData)
	{
		return mInterface.regWrite<T>(mID, aAddress, aData, mStatusReturnLevel, &mStatus);
	}
	
    OneWireStatus ping()
	{
		return mInterface.ping(mID, &mStatus);
	}
	
    OneWireStatus action()
	{
		return mInterface.action(mID, mStatusReturnLevel, &mStatus);
	}
	
    OneWireStatus reset()
	{
		return mInterface.factoryReset(mID, mStatusReturnLevel, &mStatus);
	}

    void updateId(uint8_t newId);

	
private:
	OneWireMInterface &mInterface;
	uint8_t mStatusReturnLevel;
	uint8_t mID;
	DynamixelStatus mStatus;
};


#endif
