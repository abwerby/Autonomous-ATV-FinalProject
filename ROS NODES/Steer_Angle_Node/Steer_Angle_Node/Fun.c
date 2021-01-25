
#include <avr/io.h>
#include "std_macros.h"
#include "Fun.h"

void FN()
{
	/* set PD2 and PD3 as input */
	CLR_BIT(DDRD,PD2);
	CLR_BIT(DDRD,PD3);
	
	/* Pull-up enable PD2,PD3 */
	SET_BIT(PORTD,PD2);
	SET_BIT(PORTD,PD3);
	
}