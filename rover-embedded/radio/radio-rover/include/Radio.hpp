/**
 * Copyright 2026 Comet Robotics - Wow, so professional
 * Author: Hudson Green
 * Contributors: N/A
 * Summary: Abstraction of the radio module to make it easier to work with
 *          https://github.com/jgromes/RadioLib/wiki/Basics
 *          https://jgromes.github.io/RadioLib/class_s_x127x.html
 */

#ifndef RADIO_HPP
#define RADIO_HPP

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

class Radio {

private:
    uint32_t m_pinChipSelect = 17;
    uint32_t m_pinIRQ = 1;
    uint32_t m_pinRST = 0;

    // Incremented every time sendData is run (useful for reassembly of data from multiple frames)
    uint8_t m_currBlockNo = 0;

    bool m_isMaster = true;

    SX1276 m_radio;

public:
    Radio(uint32_t chipSelectPin, uint32_t irqPin, uint32_t rstPin, SPIClassRP2040 &spi, pin_size_t sckPin, pin_size_t misoPin, pin_size_t mosiPin, const char *callsign, bool isMaster);

    bool init();

    /**
     * Encapsulates data into Frames and sends over radio
     */
    bool sendData(uint8_t *data, size_t size);
    bool sendData(const char *msg);

    /**
     * Gets frames and assembles them back into a bytestream
     */
    uint8_t *receiveData(size_t size);
};

#endif
