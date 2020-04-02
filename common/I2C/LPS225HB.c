// Copyright (c) 2018, Tobias Mueller tm(at)tm3d.de
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

//---------- Includes ----------------------------------------------------------
#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "USI_TWI_Master.h"
#include "LPS225HB.h"
typedef enum{
	I2C_ADR_W = 0b10111000, // sensor I2C address + write bit //ADR=VDD
	I2C_ADR_R = 0b10111001 // sensor I2C address + read bit
}etI2cHeader;

uint8_t LPS225HB_Init() {
	volatile uint8_t error=0;
	/*I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	error |= I2c_WriteByte (0x02); // Config
	error |= I2c_WriteByte (0x0); // 14 bit
	error |= I2c_WriteByte (0x0); // 
	I2c_StopCondition();*/


	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	error |= I2c_WriteByte (0x11); //
	//error |= I2c_WriteByte (0b01110001); //
	error |= I2c_WriteByte (0b00010100); //
	I2c_StopCondition();
	return error;

}

uint8_t LPS225HB_Readf(double * temperature, double * pressure) {	
	int16_t t;
	uint32_t p;
	uint8_t error=0;
	error= LPS225HB_Readi(&t,&p);
	*temperature=(double)t/100.0;
	*pressure=(double)p/4096.0;
	return error;

}
uint8_t readReg(uint8_t reg) {
	volatile uint8_t error=0;
	uint8_t ret;
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	error |= I2c_WriteByte (reg); //
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_R); //I2C address
	ret= I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	return ret;
}

uint8_t LPS225HB_Readi(int16_t * temperature, uint32_t * pressure) {
	volatile uint8_t error=0;
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	error |= I2c_WriteByte (0x11); //
	//error |= I2c_WriteByte (0b00000001); //
	error |= I2c_WriteByte (0b01010001); //
	I2c_StopCondition();
	_delay_ms(1000);
//	uint16_t status=0;
//	status=readReg(0x27);
//	status|=readReg(0x26)<<8;
//	*pressure=0;
//	*pressure|= readReg(0x28);
//	*pressure|= ((uint32_t)readReg(0x29))<<8;
//	*pressure|= ((uint32_t)readReg(0x2A))<<16;


	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	error |= I2c_WriteByte (0x28); //
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_R); //I2C address
//	status= I2c_ReadByte(ACK)<<8;
//	status|= I2c_ReadByte(ACK);


	*pressure=0;
	*pressure|= I2c_ReadByte(ACK);
	*pressure|= ((uint32_t)I2c_ReadByte(ACK))<<8;
	*pressure|= ((uint32_t)I2c_ReadByte(ACK))<<16;
	*temperature=0;
	*temperature |= I2c_ReadByte(ACK);
	*temperature |= ((int16_t)I2c_ReadByte(NO_ACK)<<8); 
	I2c_StopCondition();
	//*temperature=status;

	return error;
	}

