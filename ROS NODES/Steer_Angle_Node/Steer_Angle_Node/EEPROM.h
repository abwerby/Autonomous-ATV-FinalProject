/*
 * EEPROM.h
 *
 * Created: 16/01/2020 06:40:56 م
 *  Author: DR.LAP
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

void EEPROM_Write (uint16_t address, uint8_t data);
uint8_t EEPROM_Read( uint16_t address);



#endif /* EEPROM_H_ */