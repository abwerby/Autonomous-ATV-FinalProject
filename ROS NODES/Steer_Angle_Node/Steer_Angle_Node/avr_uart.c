/*
 * CFile1.c
 *
 * Created: 1/17/2020 7:02:54 PM
 *  Author: abdelrhman
 */ 

#include <avr/io.h>
#include "std_macros.h"
#include "avr_uart.h"

#define F_CPU 12000000UL
#define BAUD 57600
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1)

//#define UBRR_VAL 12
// Initialize the UART
void avr_uart_init(void)
{
	
	/* ENABLE RECEVIER */
	SET_BIT(UCSRB,RXEN);

	/* ENABLE TRANSMITER */
	SET_BIT(UCSRB,TXEN);

	/* CLEAR BIT UCSZ2 TO SET 8 BIT */
	CLR_BIT(UCSRB,UCSZ2);

	/* Asynchronous MODE */
	CLR_BIT(UCSRC,UMSEL);

	/* Parity Disabled */
	CLR_BIT(UCSRC,UPM0);
	CLR_BIT(UCSRC,UPM1);

	/* ONE BIT STOP */
	CLR_BIT(UCSRC,USBS);

	/* SET BIT UCSZ0 , UCSZ1  TO SET 8 BIT */
	SET_BIT(UCSRC,UCSZ0);
	SET_BIT(UCSRC,UCSZ1);

	/* BUAD RATE SET*/
	UBRRH = (UBRR_VAL >> 8);
	UBRRL = UBRR_VAL;
}


// Send one char (blocking)
void avr_uart_send_byte(uint8_t tx_byte)
{
	/* WAIT TELL TX BUFFER IS EMPTY */
	while(READ_BIT(UCSRA,UDRE) == 0);

	/* SET DATA REGSITER WITH DATA TO SEND */
	UDR = tx_byte ;
}


// Get one char if available, otherwise -1
int16_t avr_uart_receive_byte(void)
{
	/* WAIT TILL A VAILD BYTE AVALIBLE INSIDE RX BUFFER  */
	if(READ_BIT(UCSRA,RXC) != 0 )
	{
		/* RETURN BYTE RECIVED */
		return UDR ;
	}
	else
	{
		return -1 ;
	}
}