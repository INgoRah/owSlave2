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
#define F_CPU 8000000UL
#include <avr/io.h>

#include <util/delay.h>


#include "TWI_Master.h"
#include "CDM7160.h"




uint16_t CDM7160getCO2_() {
	volatile uint8_t b1,b2;
	while((PINA&(1<<PINA0))!=0);
	I2c_StartCondition();
	I2c_WriteByte(0b11010010);
	I2c_WriteByte(0x03);
	I2c_StartCondition();
	I2c_WriteByte (0b11010011);
	b1 =I2c_ReadByte(ACK);
	b2 =I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	
	return b1|(b2<<8);
	
	

}
uint16_t CDM7160getCO2() {
	volatile uint8_t b[16],i;
	while((PINA&(1<<PINA0))!=0);
	I2c_StartCondition();
	I2c_WriteByte(0b11010010);
	I2c_WriteByte(0x00);
	I2c_StartCondition();
	I2c_WriteByte (0b11010011);
	for(i=0;i<15;i++) {
		b[i] =I2c_ReadByte(ACK);
	}
	b[15] =I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	
	return b[3]|(b[4]<<8);
	
	

}

void CDM7160setMode(uint8_t mode) {
	while((PINA&(1<<PINA0))!=0);
	I2c_StartCondition();
	I2c_WriteByte(0b11010010);
	I2c_WriteByte(0x01);
	I2c_StartCondition();
	I2c_WriteByte (0b11010011);
	uint8_t b =I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	_delay_ms(2);
	b&=0xFC;
	b|=(mode<<1)&0x3;
	I2c_StartCondition();
	I2c_WriteByte(0b11010010);
	I2c_WriteByte(0x01);
	I2c_WriteByte(b);
	I2c_StopCondition();
}

void CDM7160softReset() {
	while((PINA&(1<<PINA0))!=0);
	I2c_StartCondition();
	I2c_WriteByte(0b11010010);
	I2c_WriteByte(0x00);
	I2c_WriteByte(0x01);
	I2c_StopCondition();
}


void CDM7160setFMode(uint8_t fmode) //Set Filtermode 0 average, 1 IIR
{
	while((PINA&(1<<PINA0))!=0);
	I2c_StartCondition();
	I2c_WriteByte(0b11010010);
	I2c_WriteByte(0x01);
	I2c_StartCondition();
	I2c_WriteByte (0b11010011);
	volatile uint8_t b =I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	_delay_ms(2);
	b&=0xFB;
	b|=((fmode&1)<<2);
	I2c_StartCondition();
	I2c_WriteByte(0b11010010);
	I2c_WriteByte(0x01);
	I2c_WriteByte(b);
	I2c_StopCondition();
}
void CDM7160setAvCount(uint8_t count) {//Set the Averaging count
	while((PINA&(	1<<PINA0))!=0);
	I2c_StartCondition();
	I2c_WriteByte(0b11010010);
	I2c_WriteByte(0x07);
	I2c_WriteByte(count&0x3F);
	I2c_StopCondition();

}; 
