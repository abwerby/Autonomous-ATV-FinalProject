#include <avr/io.h>
#include <avr/interrupt.h>
#include "std_macros.h"
#include "avr_time.h"

#define F_CPU 12000000UL
#define PRESCALER 1024UL

volatile static uint32_t overflow_ms = 0;

// The Timer2 CTC interrupt handler
ISR(TIMER2_COMP_vect)
{
    overflow_ms++;
}


/* init timer 2 in ctc mode for work as millis function (ros need it )*/
void avr_time_init(void)
{
	/* set ctc mode */
	CLR_BIT(TCCR2,WGM20);
	SET_BIT(TCCR2,WGM21);
	/* set timer count */
	OCR2 = ((F_CPU / PRESCALER) / 1000); ;
	/* set  pin OC0 mode */
	CLR_BIT(TCCR2,COM20);
	CLR_BIT(TCCR2,COM21);
	// Prescale Timer 0 to divide by 1024
	SET_BIT(TCCR2,CS20);
	SET_BIT(TCCR2,CS21);
	SET_BIT(TCCR2,CS22);
	// Enable Timer 0 overflow interrupt
	SET_BIT(TIMSK,OCIE2);
}


// Get the current time in milliseconds
uint32_t avr_time_now(void)
{
  uint32_t now;
  // Disable interrupts 
  cli();
  now = overflow_ms;
  sei();

  return now;
}