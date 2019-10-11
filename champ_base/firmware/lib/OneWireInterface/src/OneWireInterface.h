#ifndef ONE_WIRE_INTERFACE_H
#define ONE_WIRE_INTERFACE_H

#include "Arduino.h"


/** \brief Type of one-wire status code */
typedef uint8_t OneWireStatus;
/** \brief ID for broadcast */
#define OW_BROADCAST_ID 0xFE

/**
 * \brief One-wire status values
*/
enum OWStatus
{
    OW_STATUS_OK = 0, // communication was successful
    OW_STATUS_TIMEOUT = 1, // no or not enougth bytes received
    OW_STATUS_DATA_MISSING = 2, // valid packet received but without the requested data
    OW_STATUS_CHECKSUM_ERROR = 16, // packet received with invalid checksum
    OW_STATUS_COM_ERROR = 128 // other communication error
};

/**
 * \brief One-wire intruction values
*/
enum OWInstruction
{
    OW_PING = 0x01,
    OW_READ = 0x02,
    OW_WRITE = 0x03,
    OW_REG_WRITE = 0x04,
    OW_ACTION = 0x05,
    OW_FACTORY_RESET = 0x06,
    OW_SOFT_RESET = 0x07,
    OW_SYNC_WRITE = 0x83
};

/**
 * \brief One-wire status return level values
*/
enum OWReturnLevel
{
    OW_SRL_PING = 0x00,
    OW_SRL_READ = 0x01,
    OW_SRL_ALL  = 0x02
};


/**
 * \class OneWirePacket
 * \brief One-wire packet (instruction or status)
*/
class OneWirePacket
{
public:
    OneWirePacket() {}
    //note : allow to constuct from const data, but const_cast it (constness should be respected if code is correct)
    OneWirePacket(uint8_t aID, uint8_t aInstruction, uint8_t aLength = 2, const uint8_t *aData = nullptr, uint8_t aAddress = 255, uint8_t aDataLength = 255, uint8_t aIDListSize = 0, const uint8_t *aIDList = NULL) :
        mID(aID), mIDListSize(aIDListSize), mIDList(const_cast<uint8_t*>(aIDList)), mLength(aLength), mInstruction(aInstruction), mAddress(aAddress), mDataLength(aDataLength), mData(const_cast<uint8_t*>(aData))
    {
        mCheckSum = checkSum();
    }

    /** \brief Packet ID */
    uint8_t mID;
    /** \brief ID list, used for sync write, set to 0 if not used */
    uint8_t mIDListSize;
    uint8_t *mIDList;
    /** \brief Packet length (without header and ID bytes)*/
    uint8_t mLength;
    /** \brief Packet instruction or status */
    union
    {
        uint8_t mInstruction;
        uint8_t mStatus;
    };
    /** \brief Address to read/write, set to 255 if not used */
    uint8_t mAddress;
    /** \brief Length of data to read/write, only needed for read and sync write, set to 255 if not used */
    uint8_t mDataLength;
    /** \brief Pointer to packet parameter (or NULL if no parameter) */
    uint8_t *mData;
    /** \brief Packet checksum */
    uint8_t mCheckSum;

    /**
     * \brief Compute checksum of the packet
     * \return Checksum value
    */
    uint8_t checkSum();
};


class OneWireInterface
{
public:
    /**
    * \brief Start interface
    * \param[in] aBaud : Baudrate
    * \param[in] timeout : the underlying Stream will have its timeout set to this value
    *
    * Start the interface with desired baudrate, call once before using the interface
    */
    void begin(unsigned long aBaud, unsigned long timeout = 50);

    /** \brief End the underlying Stream */
    void end();

protected:
    /**
    * \brief Constructor
    * \param[in] aStream : hardare serial used by the interface
    * \param[in] aDirectionPin : direction pin, use NO_DIR_PORT if you do not one (default)
    */
    OneWireInterface(HardwareSerial &aStream, uint8_t aDirectionPin = NO_DIR_PORT);

    /**
    * \brief Send a packet on bus
    * \param[in] aPacket : Packet to send
    *
    * The function wait for the packet to be completly sent (using Stream.flush)
    */
    void sendPacket(const OneWirePacket &aPacket);

private:
    /** \brief Configure stream in half-duplex mode */
    void configureStream();

    /** \brief Switch stream to read (receive) mode */
    void readMode();

    /** \brief Switch stream to write (send) mode */
    void writeMode();

    void setReadMode();
    void setWriteMode();

public:
    static const uint8_t NO_DIR_PORT = 255;

protected:
    HardwareSerial &mStream;
    const uint8_t mDirectionPin;

};


#endif
