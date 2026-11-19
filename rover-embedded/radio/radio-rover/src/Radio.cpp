/**
 * Copyright 2026 Comet Robotics - Wow, so professional
 * Author: Hudson Green
 * Contributors: N/A
 * Summary: Abstraction of the radio module to make it easier to work with
 */

#include "Radio.hpp"
#include "Frame.hpp"

/**
 * To set up the radio. Ensure that Serial.begin() is called on the debug serial passed, but SPI is not configured
 */
Radio::Radio(uint32_t chipSelectPin, uint32_t irqPin, 
    uint32_t rstPin, SPIClassRP2040 &spi, pin_size_t sckPin, pin_size_t misoPin, 
    pin_size_t mosiPin, const char *callsign, bool isMaster
) : m_radio(new Module(chipSelectPin, irqPin, rstPin, RADIOLIB_NC, spi)) {

    // Configure SPI0 pins (omit SPI.setCS so RadioLib controls pin 17)
	spi.setSCK(sckPin);
	spi.setMISO(misoPin);
	spi.setMOSI(mosiPin);
	spi.begin();

}

bool Radio::init() {
    int16_t state = m_radio.begin(
		434.0f, // frequency (MHz)
		125.0f	// bandwidth (kHz)
	);
	if(state != RADIOLIB_ERR_NONE) {
		Serial.println("Radio init failed: ");
		Serial.println(state);
		return false;
	}

	m_radio.setGain(0); // Automatic Gain Control
    return true;
}

bool Radio::sendData(uint8_t *data, size_t size) {

    /* break data up into frames */

    int frameCount = static_cast<int>(ceil(static_cast<double>(size) / FRAME_PAYLOAD_SIZE));

    // load data into each frame and transmit it over the radio
    for(int i = 0; i < frameCount; ++i) {
        int dataOffset = i * FRAME_PAYLOAD_SIZE;
        size_t remainingBytes = size - dataOffset;

        // the size of the data we are putting into the frame
        int chunkSize = std::min(remainingBytes, FRAME_PAYLOAD_SIZE);

        // make a frame for the data
        Frame frame;
        frame.setBlockNo(m_currBlockNo);                // the current block # associated w/our data transfer
        frame.setFrameCount(frameCount - i);            // how many frames left that make up our block
        frame.setPayload(data + dataOffset, chunkSize);

        // send it over the radio
        int16_t radioTransmitState = m_radio.transmit(frame.toBytestream().data(), FRAME_PAYLOAD_SIZE);
        if (radioTransmitState == RADIOLIB_ERR_NONE) {
            // packet was sent fine
        } else {
            Serial.println("TX err: ");
            Serial.println(radioTransmitState);
            return false;
        }
    }

    // Every time we successfully transmit a payload, we increment the block number
    ++m_currBlockNo;

    return true;
}

bool Radio::sendData(const char *msg) {
    // this cast might be UB lol, just compile both modules with the same computer at the same time to avoid compiler optimizations that might break stuff
    return this->sendData(reinterpret_cast<uint8_t *>(&msg), strlen(msg));
}

std::vector<uint8_t> Radio::receiveData() {

    std::vector<uint8_t> out;

    // ensure we have enough data in buffer to read at least one packet

    // read in a packet, extract its blockNo, and the number of expected packets (frameCount), and then put its payload into the out vector buffer
    Frame currFrame;
    int blockNo = -1;

    // loop thru until we consume enough packets to meet the number of expected packets
    do {
        // read in raw data from radio
        uint8_t buff[FRAME_SIZE];
        uint16_t receiveStatus = m_radio.receive(buff, FRAME_SIZE);

        //TODO: check receive status to verify data
        //if(receiveStatus == ____) ...

        // load that raw data into a frame
        currFrame = Frame();
        currFrame.fromBytestream(buff, FRAME_SIZE);

        // load block no if it hasnt been loaded in yet (first frame received)
        if(blockNo == -1)
            blockNo = currFrame.getBlockNo();
        
        // check if received frame's block no doesn't match
        if(currFrame.getBlockNo() != blockNo) {
            // error: we received a packet that wasn't intended for this session
            Serial.println("Error: unexpected block number received - data invalid");
            return out;
        }

        // load in payload into output
        std::array<uint8_t, FRAME_PAYLOAD_SIZE> data = currFrame.getPayload();
        for(uint8_t c : data) {
            out.push_back(c);
        }

    } while(currFrame.getFrameCount() != 0);

    return out;

}
