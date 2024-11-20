// file: teensy4_usb_descriptor.c
//
// This file overrides the default Teensy 4.0 USB descriptor's product name
// and manufacturer name.
//
// For more details see:
//   teensy/avr/cores/teensy4/usb_names.h
//   teensy/avr/cores/teensy4/usb_desc.c
//   teensy/avr/cores/teensy4/usb_desc.h
//
#include <avr/pgmspace.h>
#include <usb_names.h> // teensy/avr/cores/teensy4/usb_names.h

#define PRODUCT_NAME {'D','R','I','V','E','T','R','A','I','N'}
#define PRODUCT_NAME_LEN 10
#define MANUFACTURER_NAME {'S','O','L','I','S',' ','R','O','V','E','R',' ', 'P','R','O','J','E','C','T'}
#define MANUFACTURER_NAME_LEN 19
#define SERIAL_NUMBER                                            \
	{                                                        \
		'U', 'R', 'C',  '_', 'D', 'R', 'I', 'V', 'E' \
	}
#define SERIAL_NUMBER_LEN 10
PROGMEM extern struct usb_string_descriptor_struct usb_string_manufacturer_name = {
    2 + MANUFACTURER_NAME_LEN * 2,
    3,
    MANUFACTURER_NAME
};

PROGMEM extern struct usb_string_descriptor_struct usb_string_product_name = {
    2 + PRODUCT_NAME_LEN * 2,
    3,
    PRODUCT_NAME
};

struct usb_string_descriptor_struct usb_string_serial_number = {
	2 + SERIAL_NUMBER_LEN * 2,
	3,
	SERIAL_NUMBER};