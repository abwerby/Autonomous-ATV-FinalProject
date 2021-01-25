/*
 * avr_uart.h
 *
 * Created: 1/17/2020 7:02:19 PM
 *  Author: abdelrhman
 */ 


#ifndef AVR_UART_H_
#define AVR_UART_H_

void avr_uart_init(void);
void avr_uart_send_byte(uint8_t tx_byte);
int16_t avr_uart_receive_byte(void);



#endif /* AVR_UART_H_ */