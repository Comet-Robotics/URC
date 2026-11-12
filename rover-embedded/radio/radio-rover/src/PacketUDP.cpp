/**
 * Copyright 2026 Comet Robotics - Wow, so professional
 * Author: Hudson Green
 * Contributors: Tristan Baumert, Kristen Dang
 * Summary: Main entry point for the rover radio
 */

#include "PacketUDP.hpp"

uint16_t calcChecksum(PacketUDPHeader header, std::string payload) {
    return 0;
}

PacketUDP::PacketUDP(std::string topicName, std::string payload) {
    m_topicName = topicName;
    m_payload = payload;
}

void PacketUDP::send() {
    // set the header attributes
    m_header.topicNameLength = m_topicName.size();
    m_header.length = m_payload.size();
    m_header.checksum = calcChecksum(m_header, m_payload);

    // 
}
