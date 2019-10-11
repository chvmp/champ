#include "OneWireMInterface.h"


OneWireMInterface::OneWireMInterface(HardwareSerial &aStream, uint8_t aDirectionPin) :
	OneWireInterface(aStream, aDirectionPin)
{}


OneWireStatus OneWireMInterface::receivePacket(OneWirePacket &aPacket, uint8_t answerSize)
{
	static uint8_t buffer[3];
	aPacket.mIDListSize = 0;
	aPacket.mAddress = 255;
	aPacket.mDataLength = 255;
	if (mStream.readBytes(buffer, 2) < 2)
	{
        return OW_STATUS_TIMEOUT;
	}
	if (buffer[0] != 255 || buffer[1] != 255)
	{
		return OW_STATUS_COM_ERROR;
	}
	if (mStream.readBytes(buffer, 3) < 3)
	{
		return OW_STATUS_TIMEOUT;
	}
	if (aPacket.mID != buffer[0])
	{
		return OW_STATUS_COM_ERROR;
	}
	aPacket.mLength = buffer[1];
    if (aPacket.mLength < 2)
    {
        return OW_STATUS_COM_ERROR;
    }
	aPacket.mStatus = buffer[2];
	if (aPacket.mLength > 2 && aPacket.mLength - 2 != answerSize)
	{
		return OW_STATUS_COM_ERROR;
	}
	if (aPacket.mLength > 2 && (int)mStream.readBytes(reinterpret_cast<char*>(aPacket.mData), aPacket.mLength - 2) < (aPacket.mLength - 2))
	{
		return OW_STATUS_TIMEOUT;
	}
	if (mStream.readBytes(reinterpret_cast<char*>(&(aPacket.mCheckSum)), 1) < 1)
	{
		return OW_STATUS_TIMEOUT;
	}
	if (aPacket.checkSum() != aPacket.mCheckSum)
	{
        return OW_STATUS_CHECKSUM_ERROR;
	}
    if (aPacket.mLength == 2 && answerSize != 0)
    {
        return OW_STATUS_DATA_MISSING;
    }
    return OW_STATUS_OK;
}

OneWireStatus OneWireMInterface::transaction(bool aExpectStatus, uint8_t *aSlaveStatus, uint8_t answerSize)
{
    OneWireStatus ret = OW_STATUS_OK;
    uint8_t slaveStatus = 0;
	sendPacket(mPacket);
	if(aExpectStatus)
	{
		ret = receivePacket(mPacket, answerSize);
        if (ret == OW_STATUS_OK)
        {
            slaveStatus = mPacket.mStatus;
        }
	}
    if (aSlaveStatus != nullptr)
    {
        *aSlaveStatus = slaveStatus;
    }
    return ret;
}

OneWireStatus OneWireMInterface::read(uint8_t aID, uint8_t aAddress, uint8_t aSize, uint8_t *aPtr, uint8_t aStatusReturnLevel, uint8_t *aSlaveStatus)
{
	mPacket = OneWirePacket(aID, OW_READ, 4, aPtr, aAddress, aSize);
	return transaction(aStatusReturnLevel > 0 && aID != OW_BROADCAST_ID, aSlaveStatus, aSize);
}

OneWireStatus OneWireMInterface::write(uint8_t aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel, uint8_t *aSlaveStatus)
{
	mPacket = OneWirePacket(aID, OW_WRITE, aSize+3, aPtr, aAddress);
	return transaction(aStatusReturnLevel > 1 && aID != OW_BROADCAST_ID, aSlaveStatus);
}

OneWireStatus OneWireMInterface::regWrite(uint8_t aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel, uint8_t *aSlaveStatus)
{
	mPacket = OneWirePacket(aID, OW_REG_WRITE, aSize+3, aPtr, aAddress);
	return transaction(aStatusReturnLevel > 1 && aID != OW_BROADCAST_ID, aSlaveStatus);
}

OneWireStatus OneWireMInterface::syncWrite(uint8_t nID, const uint8_t *aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel)
{
	mPacket = OneWirePacket(OW_BROADCAST_ID, OW_SYNC_WRITE, (aSize+1)*nID+4, aPtr, aAddress, aSize, nID, aID);
	return transaction(false);
}

OneWireStatus OneWireMInterface::ping(uint8_t aID, uint8_t *aSlaveStatus)
{
	mPacket = OneWirePacket(aID, OW_PING);
	return transaction(true, aSlaveStatus);
}

OneWireStatus OneWireMInterface::action(uint8_t aID, uint8_t aStatusReturnLevel, uint8_t *aSlaveStatus)
{
	mPacket = OneWirePacket(aID, OW_ACTION);
	return transaction(aStatusReturnLevel > 1 && aID != OW_BROADCAST_ID, aSlaveStatus);
}

OneWireStatus OneWireMInterface::factoryReset(uint8_t aID, uint8_t aStatusReturnLevel, uint8_t *aSlaveStatus)
{
	mPacket = OneWirePacket(aID, OW_FACTORY_RESET);
	return transaction(aStatusReturnLevel > 1 && aID != OW_BROADCAST_ID, aSlaveStatus);
}

OneWireStatus OneWireMInterface::softReset(uint8_t aID, uint8_t aStatusReturnLevel, uint8_t *aSlaveStatus)
{
    mPacket = OneWirePacket(aID, OW_SOFT_RESET);
    return transaction(aStatusReturnLevel > 1 && aID != OW_BROADCAST_ID, aSlaveStatus);
}
