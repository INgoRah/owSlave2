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
#include "SHT2xV2.h"

uint8_t initSHT2x(){

	I2c_StartCondition();
	I2c_WriteByte(0b10000000);
	I2c_WriteByte(0xFE);//Softreset 
	I2c_StopCondition();
	//Default 12 Bit RH 14 Bit T
	return 1;
}

uint8_t crcmove(uint8_t crc) {
	uint8_t bit;
	for(bit=8;bit>0;--bit) {
		if (crc&0x80) crc=(crc<<1)^  0x131;
		else crc=(crc<<1);
	}
	return crc;
}

uint8_t calcCRCSHT2x(uint8_t b1, uint8_t b2) {
	uint8_t crc=0;
	crc^=b1;
	crc=crcmove(crc);
	crc^=b2;
	return crcmove(crc);
	
}



uint8_t getSHT2xHumTemp(double *temp,double *hum) {
	uint8_t ret=1;
	I2c_StartCondition();
	I2c_WriteByte(0b10000000);
	I2c_WriteByte(0xF3); //No Hold Temp
	_delay_us(20);
	I2c_StopCondition();
	_delay_ms(85);
	I2c_StartCondition();
	I2c_WriteByte (0b10000001);
	uint8_t t1 =I2c_ReadByte(ACK);
	uint8_t t2 =I2c_ReadByte(ACK);
	uint8_t tc =I2c_ReadByte(NO_ACK);
	I2c_StopCondition();

	I2c_StartCondition();
	I2c_WriteByte(0b10000000);
	I2c_WriteByte(0xF5); //No Hold Hum
	_delay_us(20);
	I2c_StopCondition();
	_delay_ms(29);
	I2c_StartCondition();
	I2c_WriteByte (0b10000001);
	uint8_t f1 =I2c_ReadByte(ACK);
	uint8_t f2 =I2c_ReadByte(ACK);
	uint8_t fc =I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	if (calcCRCSHT2x(t1,t2)==tc)
		*temp=-46.85 + 175.72/65536 *(double)(((uint16_t)t1<<8)|(t2&0xF8));
	else ret=0;
	if (calcCRCSHT2x(f1,f2)==fc) {
		*hum=-6.0+125.0/65536*(double)(((uint16_t)f1<<8)|(f2&0xF8));
		*hum=(*hum)-((100.0/(*hum)*2.5)-2.5);
		}
	else ret=0;
	//*temp=20;
	//*hum=10;
	return ret;
}

