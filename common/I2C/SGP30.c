// Copyright (c) 2017, Tobias Mueller tm(at)tm3d.de
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//  * All advertising materials mentioning features or use of this
//    software must display the following acknowledgement: This product
//    includes software developed by tm3d.de and its contributors.
//  * Neither the name of tm3d.de nor the names of its contributors may
//    be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifdef  __4MHZ__
#define F_CPU 4000000UL
#else
#define F_CPU 8000000UL
#endif
#include <avr/io.h>


#include <util/delay.h>
#include <avr/pgmspace.h>

#include "USI_TWI_Master.h"
#include "SGP30.h"
//0x58
#define WC 0b10110000
#define RC 0b10110001
#define CRC8_POLYNOMIAL             0x31
#define CRC8_INIT                   0xFF
#define CRC8_LEN                    1


uint8_t sensirion_common_generate_crc(uint8_t *data, uint8_t count)
{
	uint16_t current_byte;
	uint8_t crc = CRC8_INIT;
	uint8_t crc_bit;

	/* calculates 8-Bit checksum with given polynomial */
	for (current_byte = 0; current_byte < count; ++current_byte) {
		crc ^= (data[current_byte]);
		for (crc_bit = 8; crc_bit > 0; --crc_bit) {
			if (crc & 0x80)
			crc = (crc << 1) ^ CRC8_POLYNOMIAL;
			else
			crc = (crc << 1);
		}
	}
	return crc;
}

int8_t sensirion_common_check_crc(uint8_t *data, uint8_t count, uint8_t checksum)
{
	if (sensirion_common_generate_crc(data, count) != checksum)
	return 0;
	return 1;
}

void readSGPXX(uint16_t com, uint8_t* data, uint8_t len) {
	I2c_StartCondition();
	I2c_WriteByte(WC);
	I2c_WriteByte(com>>8);  //Ctrl hum
	I2c_WriteByte(com&0x00FF);  //Ctrl hum
	_delay_ms(100);
	I2c_StartCondition();
	I2c_WriteByte (RC);
	for(uint8_t i=0;i<len-1;i++) {
		data[i]=I2c_ReadByte(ACK);
	}
	data[len-1]=I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
}

void writeSGPXX(uint16_t com, uint8_t* data, uint8_t len) {
	I2c_StartCondition();
	I2c_WriteByte(WC);
	I2c_WriteByte(com>>8);  //Ctrl hum
	I2c_WriteByte(com&0x00FF);  //Ctrl hum
	for(uint8_t i=0;i<len;i++) {
		I2c_WriteByte(data[i]);
	}
	I2c_StopCondition();
}





uint16_t bl1S=0,bl2S=0;

uint16_t getbl=1000;


int8_t check_convert_buf(uint8_t *b,uint16_t *v1,uint16_t *v2) {
	int8_t ret=1;
	if (sensirion_common_check_crc(b,2,b[2])) {	*v1=((int16_t)b[0])<<8|b[1];} else {*v1=0;ret=0;}
	if (sensirion_common_check_crc(b+3,2,b[5])) {*v2=((int16_t)b[3])<<8|b[4];} else {*v2=0;ret=0;}
	return ret;
}



uint16_t readEEPROM(uint8_t addr,uint16_t def) {
	uint16_t hr;
	EEARH=0;
	while(EECR & (1<<EEPE));
	EEARL=addr+1;
	EECR |= (1<<EERE);
	hr=EEDR;
	if (hr!=0xFF) {
		hr=hr<<8;
		while(EECR & (1<<EEPE));
		EEARL=addr;
		EECR |= (1<<EERE);
		hr|=EEDR;
		return hr;
	}
	return def;
}

void writeEEPROM(uint8_t addr,uint16_t val) {
	EEARH=0;
	while(EECR & (1<<EEPE));
	EECR = (0<<EEPM1)|(0<<EEPM0);
	EEARL = addr;
	EEDR = val&0xFF;
	EECR |= (1<<EEMPE);
	EECR |= (1<<EEPE);
	while(EECR & (1<<EEPE));
	EECR = (0<<EEPM1)|(0<<EEPM0);
	EEARL = addr+1;
	EEDR = val>>8;
	EECR |= (1<<EEMPE);
	EECR |= (1<<EEPE);
}

void save_baseline(uint16_t b1,uint16_t b2 ) {
	writeEEPROM(0,b1);
	writeEEPROM(2,b2);
}

void read_baseline(uint16_t *b1,uint16_t *b2) {
	*b1=readEEPROM(0,0xFFFF);
	*b2=readEEPROM(2,0xFFFF);
	*b1=0xFFFF;
	*b2=0xFFFF;
}

void set_baseline(uint16_t b1,uint16_t b2) {
	uint8_t b[10];
	b[3]=(uint16_t)b1>>8;  //irgendwie werden die Werte vertauscht zwischen lesen und schreiben
	b[4]=(uint16_t)b1&0x00FF;
	b[5]=sensirion_common_generate_crc(b+3,2);
	b[0]=(uint16_t)b2>>8;
	b[1]=(uint16_t)b2&0x00FF;
	b[2]=sensirion_common_generate_crc(b,2);
	writeSGPXX(0x201e,b,6);
	bl1S=b1;
	bl2S=b2;
}

int8_t initSGP30() {
	uint8_t b[10];
	readSGPXX(0x2032,b,3);
	_delay_ms(300);
	writeSGPXX(0x2003,0,0);
	_delay_ms(10);
	uint16_t b1=0,b2=0;
	//set_baseline(0x2210,0x3320);
	//readSGPXX(0x2015,b,6);
	read_baseline(&b1,&b2);
	if (b1!=0xFFFF) {
		set_baseline(b1,b2);
		//readSGPXX(0x2015,b,6);
		//uint16_t bl1,bl2;
		//check_convert_buf(b,&bl1,&bl2);
		
	}
	return 0x1;

}

void runSGP30(uint16_t *CO2,uint16_t *VOC,uint16_t *ETH,uint16_t *H2){
	uint8_t b[10];
	readSGPXX(0x2008,b,6);
	check_convert_buf(b,CO2,VOC);
	getbl--;
	if (getbl==0) {
		uint8_t eq=1;
		readSGPXX(0x2015,b,6);
		uint16_t bl1,bl2;
		check_convert_buf(b,&bl1,&bl2);
		int8_t bl1d=bl1-bl1S;
		int8_t bl2d=bl2-bl2S;
		if (bl1d<0) bl1d=-bl1d;
		if (bl2d<0) bl2d=-bl2d;
		if (bl1d>4) {bl1S=bl1;eq=0;}
		if (bl2d>4) {bl2S=bl2;eq=0;}
		getbl=200;  //Naechste bruefunf in 100 s
		if (eq==0) {
			*VOC+=1000;
			save_baseline(bl1,bl2);
		} 
	}
	readSGPXX(0x2050,b,6);
	check_convert_buf(b,ETH,H2);

}


