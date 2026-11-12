/**
 * Copyright 2026 Comet Robotics - Wow, so professional
 * Author: Hudson Green
 * Contributors: N/A
 * Summary: Main entry point for the rover radio
 *          https://github.com/jgromes/RadioLib/wiki/Basics
 *          https://jgromes.github.io/RadioLib/class_s_x127x.html
 */

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <cstring>

static const uint32_t PIN_CHIP_SEL = 17;
static const uint32_t PIN_IRQ = 1;
static const uint32_t PIN_RST = 0;

static const bool IS_MASTER = true;

#pragma pack(push, 1)
struct Frame {
    uint8_t controlFlags;
    uint8_t data[254];
};
#pragma pack(pop)
static_assert(sizeof(Frame) == 255);

static SX1276 module = new Module(PIN_CHIP_SEL, PIN_IRQ, PIN_RST, RADIOLIB_NC, SPI);

void setup() {
    // setup spi
    SPI.setSCK(18);
    SPI.setMISO(16);
    SPI.setMOSI(19);
    SPI.setCS(17);
    //pinMode(0, OUTPUT);
    //digitalWrite(0, LOW);    // pulldown RST (probs bad)

    // setup here
    module.gain = 0;    // 0 is automatic gain control
    module.enableOOK = false;
    module.begin(
        434.0f,     // frequency
        500.0f      // bandwidth
    );
    Serial.begin(9600);

    // Handshake
    // loop here
    if(IS_MASTER)
        module.transmit("ping");
    Frame response;
    uint8_t respBytes[sizeof(Frame)];

    module.receive(respBytes, sizeof(Frame));
    std::memcpy(&response, respBytes, sizeof(Frame));
    
    Serial.write(respBytes, sizeof(Frame));
}

void loop() {
    // Packet buffer logic here (alternate between send and receive every x packets)
    module.transmit("foo");
    Serial.write("bar");
}
