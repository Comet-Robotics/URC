#ifndef FRAME_HPP
#define FRAME_HPP

#include <string.h>

#ifndef CALLSIGN
static_assert(sizeof(int) == -1);
#endif

#ifdef CALLSIGN
static_assert(sizeof(CALLSIGN) == 7);
class Frame {
private:
    uint8_t m_controlFlags = 0b00000000;
    uint8_t m_payloadSize = 0;
    char m_data[255 - sizeof(CALLSIGN) - sizeof(m_controlFlags) - sizeof(m_payloadSize)];
    static_assert(sizeof(CALLSIGN) + sizeof(m_controlFlags) + sizeof(m_payloadSize) + sizeof(m_data) == 255);
    
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
        m_payloadSize = size;
    }

    void setPayload(uint8_t *payload, size_t size) {
        for(int i = 0; i < size; ++i) {
            m_data[i] = payload[i];
        }
        m_payloadSize = size;
    }

    uint8_t *toBytestream() {
        uint8_t out[255];
        //TODO: pack out with bytestream of frame

        return out;
    }

};
#endif

#endif
