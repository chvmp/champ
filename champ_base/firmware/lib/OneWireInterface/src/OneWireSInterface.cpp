#include "OneWireSInterface.h"

OneWireSInterface::OneWireSInterface(HardwareSerial &aStream,
    uint8_t aInstructionErrorCode, uint8_t aChecksumErrorCode,
    uint8_t aDirectionPin) :
        OneWireInterface(aStream, aDirectionPin),
        mInstructionErrorCode(aInstructionErrorCode),
        mChecksumErrorCode(aChecksumErrorCode)
{
    mPacketToSend = false;
    mInstructionTimestamp = 0;
    mID = 0xFF;
    mRDT = 0;
    mSRL = OW_SRL_ALL;
    mReceptionTimeout = OW_S_DEFAULT_RECEPTION_TIMEOUT;
    mReadCallback = nullptr;
    mWriteCallback = nullptr;
    mSoftResetCallback = nullptr;
    mFactoryResetCallback = nullptr;
    mRegWrite = false;
    mRegWriteAddr = 0;
    mRegWriteSize = 0;
}

void OneWireSInterface::communicate()
{
    sendPacketIfReady();
    if (mPacketToSend) {
        return;
    }
    while (mStream.available()) {
        if (handleNewByte(mStream.read())) {
            break;
        }
    }
    sendPacketIfReady();
}

void OneWireSInterface::sendPacketIfReady()
{
    if (mPacketToSend && micros() - mInstructionTimestamp >= mRDT) {
        sendPacket(mPacket);
        mPacketToSend = false;
    }
}

bool OneWireSInterface::handleNewByte(uint8_t b)
{
    static uint32_t lastCallTime = 0;
    static size_t rLength = 0;
    static size_t rBufferIndex = 0;
    int ret = false;
    uint32_t now = millis();

    if (rBufferIndex > 0 && now - lastCallTime > mReceptionTimeout) {
        rBufferIndex = 0;
    }
    lastCallTime = now;
    if (rBufferIndex == OW_S_RBUFFER_SIZE) {
        rBufferIndex = 0;
        ret = true;
    }
    if (rBufferIndex < 2) {
        if (b != 0xFF) {
            rBufferIndex = 0;
            return ret;
        }
    }
    else if (rBufferIndex == 2) {
        if (b == 0xFF) {
            rBufferIndex = 0;
            return ret;
        }
    }
    else if (rBufferIndex == 3) {
        if (b < 2) {
            rBufferIndex = 0;
            return ret;
        }
        rLength = (size_t)b + 4;
    }

    mRBuffer[rBufferIndex] = b;
    rBufferIndex++;

    if (rBufferIndex >= 6 && rBufferIndex == rLength) {
        mInstructionTimestamp = micros();
        handleNewPacket();
        rBufferIndex = 0;
        ret = true;
    }
    return ret;
}

void OneWireSInterface::handleNewPacket()
{
    uint8_t id = mRBuffer[2];
    uint8_t instruction = mRBuffer[4];
    size_t length = (size_t)(mRBuffer[3]) + 4;
    uint8_t checksumRef = mRBuffer[length - 1];

    if (id != mID && id != OW_BROADCAST_ID) {
        return;
    }

    if (id == OW_BROADCAST_ID) {
        mPacketToSend = false;
    }
    else if (instruction == OW_PING) {
        mPacketToSend = true;
    }
    else if (instruction == OW_READ) {
        mPacketToSend = mSRL != OW_SRL_PING;
    }
    else {
        mPacketToSend = mSRL == OW_SRL_ALL;
    }

    uint8_t checksum = 0;
    for (size_t i = 2; i < length - 1; i++) {
        checksum += mRBuffer[i];
    }
    checksum = ~checksum;
    if (checksumRef == checksum) {
        if (instruction == OW_PING) {
            if (mPacketToSend) {
                mPacket = OneWirePacket(id, mHardwareStatus);
            }
        }
        else if (instruction == OW_READ) {
            if (mPacketToSend) {
                if (mReadCallback != nullptr && length == 8) {
                    uint8_t addr = mRBuffer[5];
                    uint8_t size = mRBuffer[6];
                    /* We use mRBuffer to store the data read 
                    (it will be sent before we reuse this buffer to receive) */
                    mReadCallback(addr, size, mRBuffer);
                    mPacket = OneWirePacket(id, mHardwareStatus, size + 2, mRBuffer);
                }
                else {
                    mPacket = OneWirePacket(id, mInstructionErrorCode | mHardwareStatus);
                }
            }
        }
        else {
            bool instructionOk = false;
            uint8_t retCode = 0;
            if (instruction == OW_WRITE) {
                if (mWriteCallback != nullptr && length > 7) {
                    instructionOk = true;
                    uint8_t size = (uint8_t)(length - 7);
                    retCode = mWriteCallback(mRBuffer[5], size, &mRBuffer[6]);
                }
            }
            else if (instruction == OW_REG_WRITE) {
                if (mWriteCallback != nullptr && length > 7) {
                    instructionOk = true;
                    mRegWrite = true;
                    mRegWriteAddr = mRBuffer[5];
                    mRegWriteSize = (uint8_t)(length - 7);
                    for (size_t i = 0; i < mRegWriteSize; i++) {
                        mRegWriteData[i] = mRBuffer[i + 6];
                    }
                }
            }
            else if (instruction == OW_ACTION) {
                if (mWriteCallback != nullptr && length == 6 && mRegWrite) {
                    instructionOk = true;
                    mRegWrite = false;
                    retCode = mWriteCallback(mRegWriteAddr, mRegWriteSize, mRegWriteData);
                }
            }
            else if (instruction == OW_FACTORY_RESET) {
                if (mFactoryResetCallback != nullptr && length == 6) {
                    instructionOk = true;
                    mFactoryResetCallback();
                }
            }
            else if (instruction == OW_SOFT_RESET) {
                if (mSoftResetCallback != nullptr && length == 6) {
                    instructionOk = true;
                    mSoftResetCallback();
                }
            }
            else if (instruction == OW_SYNC_WRITE) {
                if (mWriteCallback != nullptr && length > 9 && (length - 8) % (mRBuffer[6] + 1) == 0) {
                    instructionOk = true;
                    size_t data_idx = 0;
                    for (size_t i = 7; i < length - 1; i += mRBuffer[6] + 1) {
                        if (mRBuffer[i] == mID) {
                            data_idx = i + 1;
                            break;
                        }
                    }
                    if (data_idx != 0) {
                        retCode = mWriteCallback(mRBuffer[5], mRBuffer[6], &mRBuffer[data_idx]);
                    }
                }
            }

            if (mPacketToSend) {
                uint8_t err = mHardwareStatus | retCode;
                if (!instructionOk) {
                    err |= mInstructionErrorCode;
                }
                mPacket = OneWirePacket(id, err);
            }
        }
    }
    else if (mPacketToSend) {
        mPacket = OneWirePacket(id, mChecksumErrorCode | mHardwareStatus);
    }
}

