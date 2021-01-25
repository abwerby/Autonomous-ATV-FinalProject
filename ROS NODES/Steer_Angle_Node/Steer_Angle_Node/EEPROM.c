#include <avr/io.h>
#include "std_macros.h"


void EEPROM_Write (uint16_t address, uint8_t data)
{
	while(EECR&(1<<EEWE));

	
	/* 1-Specify Address*/
	EEAR = address;
	/*2-Specify data to be written*/
	EEDR = data;
	/*3- Set Master Write Enable*/
	SET_BIT(EECR,EEMWE);
	/*4-set write enable*/
	SET_BIT(EECR,EEWE);
	
}

uint8_t EEPROM_Read(uint16_t address)
{
	
	while(EECR & (1<<EEWE));

	EEAR = address;
	SET_BIT(EECR,EERE);
	return EEDR;
}