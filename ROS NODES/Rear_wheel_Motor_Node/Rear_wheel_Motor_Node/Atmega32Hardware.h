/*
 * Atmega16Hardware.h
 *
 * Created: 1/17/2020 6:24:12 PM
 *  Author: abdelrhman
 */ 


#ifndef ATMEGA32HARDWARE_H_
#define ATMEGA32HARDWARE_H_

extern "C"
{
	#include <avr/interrupt.h>
	#include <avr/wdt.h>
	#include "avr_time.h"
	#include "avr_uart.h"
}

class Atmega32Hardware {
	public:
	Atmega32Hardware() {}


	// Initialize the AVR
	void init()
	{
		wdt_disable();
		avr_time_init();
		avr_uart_init();
		sei();
	}

	// Read a byte of data from ROS connection.
	// If no data, returns -1
	int read()
	{
		return avr_uart_receive_byte();
	}


	// Send a byte of data to ROS connection
	void write(uint8_t* data, int length)
	{
		for(int i = 0; i < length; i++)
		{
			avr_uart_send_byte(data[i]);
		}
	}


	// Returns milliseconds since start of program
	unsigned long time()
	{
		return avr_time_now();
	}
};



#endif /* ATMEGA32HARDWARE_H_ */