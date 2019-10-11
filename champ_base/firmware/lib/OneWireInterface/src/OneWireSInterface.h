#ifndef ONE_WIRE_S_INTERFACE_H
#define ONE_WIRE_S_INTERFACE_H

#include "OneWireInterface.h"

#define OW_S_RBUFFER_SIZE 256
#define OW_S_DEFAULT_RECEPTION_TIMEOUT 20 // ms


/** Callbacks prototypes definitions */
/**
* \brief Read callback
* \param[in] address : starting address for the read
* \param[in] size : number of bytes to read
* \param[out] data : array of size [size] containing the read data
*/
using OW_Read_cb = void(*)(uint8_t address, uint8_t size, uint8_t* data);
/**
* \brief Write callback
* \param[in] address : starting address for the write
* \param[in] size : number of bytes to write
* \param[in] data : array of size [size] containing the data to be written
* \return : error code to be sent to the Master
*/
using OW_Write_cb = uint8_t(*)(uint8_t address, uint8_t size, const uint8_t* data);
/**
* \brief Soft reset callback
* The actual reset must occur after this function returns and after OneWireSInterface::communicate returns.
*/
using OW_SoftReset_cb = void(*)(void);
/**
* \brief Factory reset callback
* The actual reset must occur after this function returns and after OneWireSInterface::communicate returns.
*/
using OW_FactoryReset_cb = void(*)(void);


/**
 * \class  OneWireSInterface
 * \brief Represent a one wire bus, on the slave side.
*/
class OneWireSInterface : public OneWireInterface
{
public:
    /**
    * \brief Constructor
    * \param[in] aStream : hardare serial used by the interface
    * \param[in] aInstructionErrorCode : Status value in case of Instruction Error (must be a power of 2)
    * \param[in] aChecksumErrorCode : Status value in case of Checksum Error (must be a power of 2)
    * \param[in] aDirectionPin : direction pin, use NO_DIR_PORT if you do not one (default)
    */
    OneWireSInterface(HardwareSerial &aStream, uint8_t aInstructionErrorCode,
        uint8_t aChecksumErrorCode, uint8_t aDirectionPin = NO_DIR_PORT);

    /**
     * \brief Non-blocking method to communicates with the master (send/receive). Should be called as often as possible.
    */
    void communicate();

    /**
     * \brief Setters for callbacks. If a callback is not set, instructions that need it will be answered with an Intruction Error.
    */
    void setReadCallback(OW_Read_cb cb) { mReadCallback = cb; }
    void setWriteCallback(OW_Write_cb cb) { mWriteCallback = cb; }
    void setSoftResetCallback(OW_SoftReset_cb cb) { mSoftResetCallback = cb; }
    void setFactoryResetCallback(OW_FactoryReset_cb cb) { mFactoryResetCallback = cb; }

    void setID(uint8_t id) { if (id < OW_BROADCAST_ID) mID = id; }
    uint8_t getID() const { return mID; }
    void setRDT(uint32_t rdt) { if (rdt <= 1000) mRDT = rdt; }
    uint32_t getRDT() const { return mRDT; }
    void setSRL(uint8_t srl) { if (srl <= 2) mSRL = (OWReturnLevel)srl; }
    uint8_t getSRL() const { return (uint8_t)mSRL; }
    void setReceptionTimeout(uint32_t timeout) { mReceptionTimeout = timeout; }
    uint32_t getReceptionTimeout() const { return mReceptionTimeout; }

    bool waitingToSendPacket() const { return mPacketToSend; }
    void setHardwareStatus(uint8_t hardwareStatus) { mHardwareStatus = hardwareStatus; }

private:
    void sendPacketIfReady();
    bool handleNewByte(uint8_t b); // return true if a full packet was received
    void handleNewPacket();

    OneWirePacket mPacket;
    bool mPacketToSend; // If true, the mPacket is ready to be sent
    uint32_t mInstructionTimestamp; // Timestamp of the last received instruction
    uint8_t mRBuffer[OW_S_RBUFFER_SIZE];
    uint8_t mHardwareStatus; // Status containing errors not related with the communication

    uint8_t mID; // Slave ID
    uint32_t mRDT; // Return Delay Time (µs)
    OWReturnLevel mSRL; // Status Return Level
    uint32_t mReceptionTimeout; // Maximum timespan between two consecutive bytes of a packet (ms)
    const uint8_t mInstructionErrorCode;
    const uint8_t mChecksumErrorCode;

    OW_Read_cb mReadCallback;
    OW_Write_cb mWriteCallback;
    OW_SoftReset_cb mSoftResetCallback;
    OW_FactoryReset_cb mFactoryResetCallback;

    /* Registered write memory */
    bool mRegWrite;
    uint8_t mRegWriteAddr;
    uint8_t mRegWriteSize;
    uint8_t mRegWriteData[OW_S_RBUFFER_SIZE];
};


#endif
