#include "OneWireInterface.h"


uint8_t OneWirePacket::checkSum()
{
    uint8_t result = mID + mLength + mInstruction;
    int n = 0;
    if (mAddress != 255)
    {
        result += mAddress;
        ++n;
    }
    if (mDataLength != 255)
    {
        result += mDataLength;
        ++n;
    }
    for (int i = 0; i < (mLength - 2 - n - mIDListSize); ++i)
    {
        result += mData[i];
    }
    for (int i = 0; i < mIDListSize; ++i)
    {
        result += mIDList[i];
    }
    return ~result;
}


OneWireInterface::OneWireInterface(HardwareSerial &aStream, uint8_t aDirectionPin) :
    mStream(aStream), mDirectionPin(aDirectionPin)
{
    if (mDirectionPin != NO_DIR_PORT)
    {
        digitalWrite(mDirectionPin, LOW);
        pinMode(mDirectionPin, OUTPUT);
    }
}

void OneWireInterface::begin(unsigned long aBaud, unsigned long timeout)
{
    mStream.begin(aBaud);

    if (mDirectionPin == NO_DIR_PORT)
    {
        configureStream();
    }
    mStream.setTimeout(timeout); //warning : response delay seems much higher than expected for some operation (eg writing eeprom)
    readMode();
}

void OneWireInterface::end()
{
    mStream.end();
}

void OneWireInterface::sendPacket(const OneWirePacket &aPacket)
{
    writeMode();

    mStream.write(0xFF);
    mStream.write(0xFF);
    mStream.write(aPacket.mID);
    mStream.write(aPacket.mLength);
    mStream.write(aPacket.mInstruction);
    uint8_t n = 0;
    if (aPacket.mAddress != 255)
    {
        mStream.write(aPacket.mAddress);
        ++n;
    }
    if (aPacket.mDataLength != 255)
    {
        mStream.write(aPacket.mDataLength);
        ++n;
    }
    if (aPacket.mLength > (2 + n) && aPacket.mData != nullptr)
    {
        if (aPacket.mIDListSize == 0)
        {
            mStream.write(aPacket.mData, aPacket.mLength - 2 - n);
        }
        else
        {
            uint8_t *ptr = aPacket.mData;
            for (uint8_t i = 0; i < aPacket.mIDListSize; ++i)
            {
                mStream.write(aPacket.mIDList[i]);
                mStream.write(ptr, aPacket.mDataLength);
                ptr += aPacket.mDataLength;
            }
        }
    }
    mStream.write(aPacket.mCheckSum);
    mStream.flush();
#ifdef TEENSYDUINO
    mStream.clear();
#else
    while (mStream.available())
    {
        mStream.read();
    }
#endif
    readMode();
}

void OneWireInterface::readMode()
{
    if (mDirectionPin != NO_DIR_PORT)
    {
        digitalWrite(mDirectionPin, LOW);
    }
    else
    {
        setReadMode();
    }
}

void OneWireInterface::writeMode()
{
    if (mDirectionPin != NO_DIR_PORT)
    {
        digitalWrite(mDirectionPin, HIGH);
    }
    else
    {
        setWriteMode();
    }
}


/* ---------------------------------------------- */
/* UART configuration for all supported platforms */
/* ---------------------------------------------- */

#ifdef __AVR__

// define TXEN, RXEN and RXCIE
#if !defined(TXEN)
#if defined(TXEN0)
#define TXEN TXEN0
#define RXEN RXEN0
#define RXCIE RXCIE0
#elif defined(TXEN1) // Some devices have uart1 but no uart0 (leonardo)
#define TXEN TXEN1
#define RXEN RXEN1
#define RXCIE RXCIE1
#endif
#endif

// determine txpin number from hardware serial interface
static uint8_t TxPinFromHardwareSerial(const HardwareSerial &aSerial)
{
#if \
defined ARDUINO_AVR_UNO || \
defined ARDUINO_AVR_DUEMILANOVE || \
defined ARDUINO_AVR_NANO || \
defined ARDUINO_AVR_MEGA2560 || \
defined ARDUINO_AVR_MEGA || \
defined ARDUINO_AVR_ADK || \
defined ARDUINO_AVR_MINI || \
defined ARDUINO_AVR_ETHERNET || \
defined ARDUINO_AVR_BT || \
defined ARDUINO_AVR_PRO
    if (&aSerial == &Serial) {
        return 1;
    }
#if \
defined ARDUINO_AVR_MEGA2560 || \
defined ARDUINO_AVR_MEGA || \
defined ARDUINO_AVR_ADK 
    if (&aSerial == &Serial1) {
        return 18;
    }
    if (&aSerial == &Serial2) {
        return 16;
    }
    if (&aSerial == &Serial3) {
        return 14;
    }
#endif
#elif \
defined ARDUINO_AVR_LEONARDO || \
defined ARDUINO_AVR_MICRO || \
defined ARDUINO_AVR_ROBOT_CONTROL || \
defined ARDUINO_ARC32_TOOLS
    if (&aSerial == &Serial1) {
        return 1;
    }
#else
#error One-wire interface: unsupported hardware
#endif
    return -1;
}

// dirty trick to access protected member
class HardwareSerialAccess : public HardwareSerial
{
public:
    volatile uint8_t * const ucsrb() { return _ucsrb; }
};

#endif //__AVR__


void OneWireInterface::configureStream()
{
#if \
/* Teensy 3.0     */ defined(__MK20DX128__) || \
/* Teensy 3.1/3.2 */ defined(__MK20DX256__) || \
/* Teensy 3.5     */ defined(__MK64FX512__) || \
/* Teensy 3.6     */ defined(__MK66FX1M0__)
    if (&mStream == &Serial1)
    {
        UART0_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
        CORE_PIN1_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
    }
    else if (&mStream == &Serial2)
    {
        UART1_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
        CORE_PIN10_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
    }
    else if (&mStream == &Serial3)
    {
        UART2_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
        CORE_PIN8_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
    }
#if \
/* Teensy 3.5 */ defined(__MK64FX512__) || \
/* Teensy 3.6 */ defined(__MK66FX1M0__)
    else if (&mStream == &Serial4)
    {
        UART3_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
        CORE_PIN32_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
    }
    else if (&mStream == &Serial5)
    {
        UART4_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
        CORE_PIN33_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
    }
    else if (&mStream == &Serial6)
    {
        UART5_C1 |= UART_C1_LOOPS | UART_C1_RSRC; // Connect internally RX and TX for half duplex
        CORE_PIN48_CONFIG |= PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin
    }
#endif
#elif \
/* Teensy 4.0 */ defined(__IMXRT1062__)
    /* TODO */
#elif defined(__AVR__)
    /* Nothing to do for AVR */
#else
#error One-wire interface: unsupported hardware
#endif
}

void OneWireInterface::setReadMode()
{
#if \
/* Teensy 3.0     */ defined(__MK20DX128__) || \
/* Teensy 3.1/3.2 */ defined(__MK20DX256__) || \
/* Teensy 3.5     */ defined(__MK64FX512__) || \
/* Teensy 3.6     */ defined(__MK66FX1M0__)
    if (&mStream == &Serial1)
    {
        UART0_C3 &= ~UART_C3_TXDIR;
    }
    else if (&mStream == &Serial2)
    {
        UART1_C3 &= ~UART_C3_TXDIR;
    }
    else if (&mStream == &Serial3)
    {
        UART2_C3 &= ~UART_C3_TXDIR;
    }
#if \
/* Teensy 3.5 */ defined(__MK64FX512__) || \
/* Teensy 3.6 */ defined(__MK66FX1M0__)
    else if (&mStream == &Serial4)
    {
        UART3_C3 &= ~UART_C3_TXDIR;
    }
    else if (&mStream == &Serial5)
    {
        UART4_C3 &= ~UART_C3_TXDIR;
    }
    else if (&mStream == &Serial6)
    {
        UART5_C3 &= ~UART_C3_TXDIR;
    }
#endif
#elif \
/* Teensy 4.0 */ defined(__IMXRT1062__)
    /* TODO */
#elif defined(__AVR__)
    static HardwareSerialAccess &stream = reinterpret_cast<HardwareSerialAccess&>(mStream);
    static uint8_t TxPin = TxPinFromHardwareSerial(mStream);
    *(stream.ucsrb()) &= ~_BV(TXEN);
    *(stream.ucsrb()) |= _BV(RXEN);
    *(stream.ucsrb()) |= _BV(RXCIE);
    pinMode(TxPin, INPUT_PULLUP);
#else
#error One-wire interface: unsupported hardware
#endif
}

void OneWireInterface::setWriteMode()
{
#if \
/* Teensy 3.0     */ defined(__MK20DX128__) || \
/* Teensy 3.1/3.2 */ defined(__MK20DX256__) || \
/* Teensy 3.5     */ defined(__MK64FX512__) || \
/* Teensy 3.6     */ defined(__MK66FX1M0__)
    if (&mStream == &Serial1)
    {
        UART0_C3 |= UART_C3_TXDIR;
    }
    else if (&mStream == &Serial2)
    {
        UART1_C3 |= UART_C3_TXDIR;
    }
    else if (&mStream == &Serial3)
    {
        UART2_C3 |= UART_C3_TXDIR;
    }
#if \
/* Teensy 3.5 */ defined(__MK64FX512__) || \
/* Teensy 3.6 */ defined(__MK66FX1M0__)
    else if (&mStream == &Serial4)
    {
        UART3_C3 |= UART_C3_TXDIR;
    }
    else if (&mStream == &Serial5)
    {
        UART4_C3 |= UART_C3_TXDIR;
    }
    else if (&mStream == &Serial6)
    {
        UART5_C3 |= UART_C3_TXDIR;
    }
#endif
#elif \
/* Teensy 4.0 */ defined(__IMXRT1062__)
    /* TODO */
#elif defined(__AVR__)
    static HardwareSerialAccess &stream = reinterpret_cast<HardwareSerialAccess&>(mStream);
    *(stream.ucsrb()) &= ~_BV(RXEN);
    *(stream.ucsrb()) &= ~_BV(RXCIE);
    *(stream.ucsrb()) |= _BV(TXEN);
#else
#error One-wire interface: unsupported hardware
#endif
}
