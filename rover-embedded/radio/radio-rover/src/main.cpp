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

static constexpr uint32_t PIN_CHIP_SEL = 17;
static constexpr uint32_t PIN_IRQ = 1;
static constexpr uint32_t PIN_RST = 0;

static constexpr bool IS_MASTER = true;

#pragma pack(push, 1)
struct Frame {
	const char *callsign = "KM7AYC";
	uint8_t controlFlags = 0;
	uint8_t data[255 - sizeof(callsign) - sizeof(controlFlags)] = { 0 };
};
#pragma pack(pop)
static_assert(sizeof(Frame) == 255);

static Module loraMod(PIN_CHIP_SEL, PIN_IRQ, PIN_RST, RADIOLIB_NC, SPI);
static SX1276 radio(&loraMod);

void setup() {
	Serial.begin(115200);
	while(!Serial && millis() < 2000);

	// Configure SPI0 pins (omit SPI.setCS so RadioLib controls pin 17)
	SPI.setSCK(18);
	SPI.setMISO(16);
	SPI.setMOSI(19);
	SPI.begin();

	int16_t state = radio.begin(
		434.0f, // frequency (MHz)
		125.0f	// bandwidth (kHz)
	);
	if(state != RADIOLIB_ERR_NONE) {
		Serial.print("Radio init failed: ");
		Serial.println(state);
		while(true) {
			delay(100);
		}
	}

	radio.setGain(0); // Automatic Gain Control

	// Handshake
	if(IS_MASTER) {
		radio.transmit("KM7AYC: ping");

		Frame response{};
		int16_t rxState = radio.receive(reinterpret_cast<uint8_t *>(&response), sizeof(Frame));
		if(rxState == RADIOLIB_ERR_NONE) {
			size_t len = radio.getPacketLength();
			Serial.write(reinterpret_cast<uint8_t *>(&response), len);
		} else {
			Serial.print("Handshake timeout/err: ");
			Serial.println(rxState);
		}
	}
}

void loop() {
	Frame transmit = Frame();
	transmit.data[0] = 5;
	int16_t txState = radio.transmit("KM7AYC: module transmit");
	if (txState == RADIOLIB_ERR_NONE) {
		Serial.println("TX OK");
	} else {
		Serial.print("TX err: ");
		Serial.println(txState);
	}

	delay(20); // Prevent 100% duty cycle / PA overheating
}
