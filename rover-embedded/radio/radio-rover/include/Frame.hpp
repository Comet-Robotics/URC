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

class Frame {
private:
    uint8_t m_frameNo = 0;
    uint8_t m_payloadSize = 0;
    char m_data[FRAME_PAYLOAD_SIZE];
    static_assert(sizeof(m_frameNo) + sizeof(m_payloadSize) + sizeof(m_data) == FRAME_SIZE);
    
public:
    Frame() {}

    Frame(const char *payload) {
        setPayload(payload);
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
        m_payloadSize = size;
    }

    void setPayload(uint8_t *payload, size_t size) {
        for(int i = 0; i < size; ++i) {
            m_data[i] = payload[i];
        }
        for(int i = size; i < FRAME_PAYLOAD_SIZE; ++i) {
            m_data[i] = 0;
        }
        m_payloadSize = size;
    }

    std::array<uint8_t, FRAME_SIZE> toBytestream() {
        std::array<uint8_t, FRAME_SIZE> out;

        // header info
        out[0] = m_frameNo;
        out[1] = m_payloadSize;
        const int PAYLOAD_START_BYTE = 2;

        // payload
        for(int i = PAYLOAD_START_BYTE; i < FRAME_SIZE; ++i) {
            out[i] = m_data[i - PAYLOAD_START_BYTE];
        }

        return out;
    }

    /*void fromBytestream(uint8_t *data, size_t size) {
        //TODO: get data and load it into frame
    }*/

};

#endif
