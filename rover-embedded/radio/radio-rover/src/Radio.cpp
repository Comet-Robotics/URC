/**
 * Copyright 2026 Comet Robotics - Wow, so professional
 * Author: Hudson Green
 * Contributors: N/A
 * Summary: Abstraction of the radio module to make it easier to work with
 */

#include "Radio.hpp"
#include "Frame.hpp"

void Radio::log(const char *msg) {
    m_debugSerial->print(msg);
}

void Radio::log(int16_t msg) {
    m_debugSerial->print(msg);
}

/**
 * To set up the radio. Ensure that Serial.begin() is called on the debug serial passed, but SPI is not configured
 */
Radio::Radio(HardwareSerial *debugSerialOut, uint32_t chipSelectPin, uint32_t irqPin, 
    uint32_t rstPin, SPIClassRP2040 &spi, pin_size_t sckPin, pin_size_t misoPin, 
    pin_size_t mosiPin, char *callsign, bool isMaster
) : 
m_debugSerial(debugSerialOut), 
m_radio(new Module(chipSelectPin, irqPin, rstPin, RADIOLIB_NC, spi)) {

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
		log("Radio init failed: ");
		log(state);
		return false;
	}

	m_radio.setGain(0); // Automatic Gain Control
    return true;
}

bool Radio::sendData(uint8_t *data, size_t size) {

    /* break data up into frames */

    int frameCount = static_cast<int>(ceil(static_cast<double>(size) / FRAME_PAYLOAD_SIZE));

    Frame frames[frameCount];

    // load data into each frame and transmit it over the radio
    for(int i = 0; i < frameCount; ++i) {
        int dataOffset = i * FRAME_PAYLOAD_SIZE;
        size_t remainingBytes = size - dataOffset;

        // the size of the data we are putting into the frame
        int chunkSize = std::min(remainingBytes, FRAME_PAYLOAD_SIZE);

        // make a frame for the data
        Frame frame;
        frame.setPayload(data + dataOffset, chunkSize);

        // send it over the radio
        int16_t radioTransmitState = m_radio.transmit(frame.toBytestream().data(), FRAME_PAYLOAD_SIZE);
        if (radioTransmitState == RADIOLIB_ERR_NONE) {
            // packet was sent fine
        } else {
            log("TX err: ");
            log(radioTransmitState);
            return false;
        }
    }

    return true;
}

bool Radio::sendData(const char *msg) {
    // this cast might be UB lol
    return this->sendData(reinterpret_cast<uint8_t *>(&msg), strlen(msg));
}

uint8_t *Radio::receiveData(size_t size) {
    return nullptr;
}
