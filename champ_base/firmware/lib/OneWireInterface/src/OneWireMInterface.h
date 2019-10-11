#ifndef ONE_WIRE_M_INTERFACE_H
#define ONE_WIRE_M_INTERFACE_H

#include "OneWireInterface.h"


/**
 * \class  OneWireMInterface
 * \brief Represent a one wire bus, on the master side.
*/
class OneWireMInterface : public OneWireInterface
{
public:
	/**
	* \brief Constructor
	* \param[in] aStream : hardare serial used by the interface
	* \param[in] aDirectionPin : direction pin, use NO_DIR_PORT if you do not one (default)
	*/
	OneWireMInterface(HardwareSerial &aStream, uint8_t aDirectionPin = NO_DIR_PORT);
	
	// sizeof(T) must be strictly lower than 254
	template<class T>
	inline OneWireStatus read(uint8_t aID, uint8_t aAddress, T& aData, uint8_t aStatusReturnLevel = 2, uint8_t *aSlaveStatus = nullptr);
	template<class T>
	inline OneWireStatus write(uint8_t aID, uint8_t aAddress, const T& aData, uint8_t aStatusReturnLevel = 2, uint8_t *aSlaveStatus = nullptr);
	template<class T>
	inline OneWireStatus regWrite(uint8_t aID, uint8_t aAddress, const T& aData, uint8_t aStatusReturnLevel = 2, uint8_t *aSlaveStatus = nullptr);

    OneWireStatus read(uint8_t aID, uint8_t aAddress, uint8_t aSize, uint8_t *aPtr, uint8_t aStatusReturnLevel = 2, uint8_t *aSlaveStatus = nullptr);
    OneWireStatus write(uint8_t aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel = 2, uint8_t *aSlaveStatus = nullptr);
    OneWireStatus regWrite(uint8_t aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel = 2, uint8_t *aSlaveStatus = nullptr);
    OneWireStatus syncWrite(uint8_t nID, const uint8_t *aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel = 2);

    OneWireStatus ping(uint8_t aID, uint8_t *aSlaveStatus = nullptr);
    OneWireStatus action(uint8_t aID=OW_BROADCAST_ID, uint8_t aStatusReturnLevel = 2, uint8_t *aSlaveStatus = nullptr);
    OneWireStatus factoryReset(uint8_t aID, uint8_t aStatusReturnLevel=2, uint8_t *aSlaveStatus = nullptr);
    OneWireStatus softReset(uint8_t aID, uint8_t aStatusReturnLevel=2, uint8_t *aSlaveStatus = nullptr);

private:
    /**
    * \brief Receive a packet on bus
    * \param[out] aPacket : Received packet. mData field must be previously allocated
    *
    * The function wait for a new packet on the bus. Timeout depends of timeout of the underlying stream.
    * Return error code in case of communication error (timeout, checksum error, ...)
    */
    OneWireStatus receivePacket(OneWirePacket &aPacket, uint8_t answerSize = 0);

    OneWireStatus transaction(bool aExpectStatus, uint8_t *aSlaveStatus = nullptr, uint8_t answerSize = 0);

    OneWirePacket mPacket;
};


template<class T>
OneWireStatus OneWireMInterface::read(uint8_t aID, uint8_t aAddress, T& aData, uint8_t aStatusReturnLevel, uint8_t *aSlaveStatus)
{
	return read(aID, aAddress, uint8_t(sizeof(T)), (uint8_t*)&aData, aStatusReturnLevel, aSlaveStatus);
}

template<class T>
OneWireStatus OneWireMInterface::write(uint8_t aID, uint8_t aAddress, const T& aData, uint8_t aStatusReturnLevel, uint8_t *aSlaveStatus)
{
	return write(aID, aAddress, uint8_t(sizeof(T)), (const uint8_t*)&aData, aStatusReturnLevel, aSlaveStatus);
}

template<class T>
OneWireStatus OneWireMInterface::regWrite(uint8_t aID, uint8_t aAddress, const T& aData, uint8_t aStatusReturnLevel, uint8_t *aSlaveStatus)
{
	return regWrite(aID, aAddress, uint8_t(sizeof(T)), (const uint8_t*)aData, aStatusReturnLevel, aSlaveStatus);
}


#endif
