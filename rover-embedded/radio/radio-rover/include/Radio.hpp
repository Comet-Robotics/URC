/**
 * Copyright 2026 Comet Robotics - Wow, so professional
 * Author: Hudson Green
 * Contributors: N/A
 * Summary: Abstraction of the radio module to make it easier to work with
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

    bool m_isMaster = true;

    HardwareSerial *m_debugSerial;

    SX1276 m_radio;

    void log(const char *msg);
    void log(int16_t msg);

public:
    Radio(HardwareSerial *debugSerialOut, uint32_t chipSelectPin, uint32_t irqPin, uint32_t rstPin, SPIClassRP2040 &spi, pin_size_t sckPin, pin_size_t misoPin, pin_size_t mosiPin, char *callsign, bool isMaster);

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
