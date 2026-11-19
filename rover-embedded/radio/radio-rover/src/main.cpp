/**
 * Copyright 2026 Comet Robotics - Wow, so professional
 * Author: Hudson Green
 * Contributors: N/A
 * Summary: Main entry point for the rover radio
 */

#include <Radio.hpp>

static constexpr uint32_t PIN_CHIP_SEL = 17;
static constexpr uint32_t PIN_IRQ = 1;
static constexpr uint32_t PIN_RST = 0;

static constexpr bool IS_MASTER = true;

static Radio *radio;

void setup() {
	Serial.begin(115200);
	while(!Serial && millis() < 2000);
	
	radio = new Radio(PIN_CHIP_SEL, PIN_IRQ, PIN_RST, SPI, 18, 16, 19, "K5UTD ", IS_MASTER);
	radio->init();
	
}

void loop() {
	radio->sendData("transmit data");

	delay(20); // Prevent 100% duty cycle / PA overheating
}
