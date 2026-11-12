/**
 * Copyright 2026 Comet Robotics - Wow, so professional
 * Author: Hudson Green
 * Contributors: N/A
 * Summary: Main entry point for the rover radio
 */

#ifndef PACKETUDP_HPP
#define PACKETUDP_HPP

#include <cstdint>
#include <string>

#pragma pack(push, 1)
struct PacketUDPHeader {
    uint16_t length;
    uint16_t checksum;
    uint8_t topicNameLength;
};
#pragma pack(pop)
static_assert(sizeof(PacketUDPHeader) == 5);

class PacketUDP {
protected:
    PacketUDPHeader m_header;
    std::string m_topicName;
    std::string m_payload;
public:
    PacketUDP(std::string topicName, std::string payload);
    void send();
};

#endif
