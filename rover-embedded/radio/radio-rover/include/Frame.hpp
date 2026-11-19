/**
 * Copyright 2026 Comet Robotics - Wow, so professional
 * Author: Hudson Green
 * Contributors: N/A
 * Summary: Frame class for the radio module
 */

#ifndef FRAME_HPP
#define FRAME_HPP

#define FRAME_SIZE 255
#define FRAME_PAYLOAD_SIZE 255 - sizeof(uint16_t)

#include <array>
#include <cstring>

class Frame {
private:
    uint8_t m_blockNo = 0;          // the number associated with the data being transmitted
    uint8_t m_frameCount = 0;       // how many frames left associated with this block no
    char m_data[FRAME_PAYLOAD_SIZE];
    static_assert(sizeof(m_blockNo) + sizeof(m_frameCount) + sizeof(m_data) == FRAME_SIZE);
    
public:
    Frame() {}

    Frame(const char *payload) {
        setPayload(payload);
    }

    void setBlockNo(uint8_t frameNo) {
        m_blockNo = frameNo;
    }

    void setFrameCount(uint8_t frameCt) {
        m_frameCount = frameCt;
    }

    void setPayload(const char *payload) {
        size_t size = 0;
        for(int i = 0; i < 255 && payload[i] != '\0'; ++i) {
            m_data[i] = payload[i];
            ++size;
        }
        for(int i = size; i < FRAME_PAYLOAD_SIZE; ++i) {
            m_data[i] = 0;
        }
    }

    void setPayload(uint8_t *payload, size_t size) {
        for(int i = 0; i < size; ++i) {
            m_data[i] = payload[i];
        }
        for(int i = size; i < FRAME_PAYLOAD_SIZE; ++i) {
            m_data[i] = 0;
        }
    }

    uint8_t getBlockNo() {
        return m_blockNo;
    }

    uint8_t getFrameCount() {
        return m_frameCount;
    }

    std::array<uint8_t, FRAME_PAYLOAD_SIZE> getPayload() {
        std::array<uint8_t, FRAME_PAYLOAD_SIZE> out;
        std::memcpy(out.data(), m_data, FRAME_PAYLOAD_SIZE);
        return out;
    }

    std::array<uint8_t, FRAME_SIZE> toBytestream() {
        std::array<uint8_t, FRAME_SIZE> out;

        // header info
        out[0] = m_blockNo;
        out[1] = m_frameCount;
        const int PAYLOAD_START_BYTE = 2;

        // payload
        for(int i = PAYLOAD_START_BYTE; i < FRAME_SIZE; ++i) {
            out[i] = m_data[i - PAYLOAD_START_BYTE];
        }

        return out;
    }

    void fromBytestream(uint8_t *data, size_t size) {
        if(size != FRAME_SIZE) {
            Serial.println("Error: Invalid size specified. Must be equal to FRAME_SIZE (typically 255 bytes)");
            return;
        }
        // get data and load it into frame
        this->m_blockNo = data[0];
        this->m_frameCount = data[1];
        const int PAYLOAD_START_BYTE = 2;

        // load in payload
        for(int i = PAYLOAD_START_BYTE; i < FRAME_SIZE; ++i) {
            this->m_data[i - PAYLOAD_START_BYTE] = data[i];
        }
    }

};

#endif
