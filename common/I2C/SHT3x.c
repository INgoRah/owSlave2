//==============================================================================
// S E N S I R I O N AG, Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project : SHT2x Sample Code (V1.2)
// File : SHT2x.c
// Author : MST
// Controller: NEC V850/SG3 (uPD70F3740)
// Compiler : IAR compiler for V850 (3.50A)
// Brief : Sensor layer. Functions for sensor access
//==============================================================================
//---------- Includes ----------------------------------------------------------
#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "TWI_Master.h"
#include "SHT3x.h"

uint8_t initSHT3x(uint8_t adrline){
	/*I2c_StartCondition();
	I2c_WriteByte(0b10001000|(adrline<<1));
	I2c_WriteByte(0x27);
	I2c_WriteByte(0x37);
	I2c_StopCondition();*/
	return 1;
}

uint8_t calcCRCSHT3x(uint8_t b1, uint8_t b2) {
	uint8_t bit;
	uint8_t crc=0xFF;
	crc^=b1;
	for(bit=8;bit>0;--bit) {
		if (crc&0x80) crc=(crc<<1)^  0x131;
		else crc=(crc<<1);
	}
	crc^=b2;
	for(bit=8;bit>0;--bit) {
		if (crc&0x80) crc=(crc<<1)^  0x131;
		else crc=(crc<<1);
	}
	return crc;
	
}



uint8_t getSHT3xHumTemp(uint8_t adrline,double *temp,double *hum) {
	uint8_t ret=1;
	I2c_StartCondition();
	I2c_WriteByte(0b10001000|(adrline<<1));
	I2c_WriteByte(0x24);
	I2c_WriteByte(0x00); //NotClock Scr
	I2c_StopCondition();
	_delay_ms(16);
	
	I2c_StartCondition();
	I2c_WriteByte (0b10001001|(adrline<<1));
	volatile uint8_t t1 =I2c_ReadByte(ACK);
	volatile uint8_t t2 =I2c_ReadByte(ACK);
	volatile uint8_t tc =I2c_ReadByte(ACK);
	volatile uint8_t f1 =I2c_ReadByte(ACK);
	volatile uint8_t f2 =I2c_ReadByte(ACK);
	volatile uint8_t fc =I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	if (calcCRCSHT3x(t1,t2)==tc)
		*temp=175.0f*(double)(((uint16_t)t1<<8)|t2)/65535.0f-45.0f;
	else ret=0;
	if (calcCRCSHT3x(f1,f2)==fc)
		*hum=100.0f*(double)(((uint16_t)f1<<8)|f2)/65535.0f;
	else ret=0;
	return ret;
}
