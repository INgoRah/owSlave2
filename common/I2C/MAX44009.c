// Copyright (c) 2015, Tobias Mueller tm(at)tm3d.de
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
#include "MAX44009.h"

uint8_t checkMAX44009(uint8_t nr) {
	volatile uint8_t b1;
	nr=(nr<<1)&0x02f;
	
	I2c_StartCondition();
	I2c_WriteByte(0b10010100|nr);
	I2c_WriteByte(0x03);
	I2c_StartCondition();
	I2c_WriteByte (0b10010101|nr);
	b1 =I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	return b1!=0xFF;
	
}


double MAX44009getlux(uint8_t nr)  {
	volatile uint8_t b1,b2;
	nr=(nr<<1)&0x02f;
	
	I2c_StartCondition();
	I2c_WriteByte(0b10010100|nr);
	I2c_WriteByte(0x03);
	I2c_StartCondition();
	I2c_WriteByte (0b10010101|nr);
	b1 =I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	I2c_StartCondition();
	I2c_WriteByte(0b10010100|nr);
	I2c_WriteByte(0x04);
	I2c_StartCondition();
	I2c_WriteByte (0b10010101|nr);
	b2 =I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	uint8_t exponent = (b1 & 0xF0) >> 4;// upper four bits of high byte register
	uint8_t mantissa = (b1 & 0x0F) << 4;// lower four bits of high byte register =
	// upper four bits of mantissa
	mantissa += b2 & 0x0F; 	  // lower four bits of low byte register =
	// lower four bits of mantissa
	
	return (double)mantissa * (double)(1 << (uint16_t)exponent) * 0.045;
	
	

}
