// Copyright (c) 2016, Tobias Mueller tm(at)tm3d.de
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
#include "HDC1080.h"
typedef enum{
	I2C_ADR_W = 128, // sensor I2C address + write bit
	I2C_ADR_R = 129 // sensor I2C address + read bit
}etI2cHeader;

uint8_t HDC1080_Init() {
	volatile uint8_t error=0;
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	error |= I2c_WriteByte (0x02); // Config
	error |= I2c_WriteByte (0x0); // 14 bit
	error |= I2c_WriteByte (0x0); // 
	I2c_StopCondition();
	return error;

}

uint8_t HDC1080_Readf(double * temperature, double * hum) {	
	int16_t t;
	uint16_t h;
	uint8_t error=0;
	error=HDC1080_Readi(&t,&h);
	*temperature=(double)t/65536.0*165.0-40.0;
	*hum=(double)h/65536.0*100;
	return error;

}
uint8_t HDC1080_Readi(int16_t * temperature, uint16_t * hum) {
	volatile uint8_t error=0;
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	error |= I2c_WriteByte (0x0); //
	I2c_StopCondition();
	_delay_ms(10);
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_R); //I2C address
	*temperature=0;
	*temperature |= I2c_ReadByte(ACK)<<8;
	*temperature |= I2c_ReadByte(ACK); 
	I2c_StopCondition();

	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	error |= I2c_WriteByte (0x01); //
	I2c_StopCondition();
	_delay_ms(10);
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_R); //I2C address
	*hum=0;
	*hum|= I2c_ReadByte(ACK)<<8; 
	*hum|= I2c_ReadByte(NO_ACK); 
	I2c_StopCondition();
	return error;
	}

/*

const uint16_t POLYNOMIAL = 0x131; //P(x)=x^8+x^5+x^4+1 = 100110001


//==============================================================================
uint8_t SHT2x_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
//==============================================================================
{
	uint8_t crc = 0;
	uint8_t byteCtr;
	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
	{ crc ^= (data[byteCtr]);
		for (uint8_t bit = 8; bit > 0; --bit)
		{ if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	if (crc != checksum) return CHECKSUM_ERROR;
	else return 0;
}
//===========================================================================
uint8_t SHT2x_ReadUserRegister(uint8_t *pRegisterValue)
//===========================================================================
{
	uint8_t checksum; //variable for checksum byte
	uint8_t error=0; //variable for error code
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W);
	error |= I2c_WriteByte (USER_REG_R);
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_R);
	*pRegisterValue = I2c_ReadByte(ACK);
	checksum=I2c_ReadByte(NO_ACK);
	error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
	I2c_StopCondition();
	return error;
}
//===========================================================================
uint8_t SHT2x_WriteUserRegister(uint8_t *pRegisterValue)
//===========================================================================
{
	uint8_t error=0; //variable for error code
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W);
	error |= I2c_WriteByte (USER_REG_W);
	error |= I2c_WriteByte (*pRegisterValue);
	I2c_StopCondition();
	return error;
}
//===========================================================================
uint8_t SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, int16_t *pMeasurand)
//===========================================================================
{
	uint8_t checksum; //checksum
	uint8_t data[2]; //data array for checksum verification
	uint8_t error=0; //error variable
	uint16_t i; //counting variable
	//-- write I2C sensor address and command --
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	switch(eSHT2xMeasureType)
	{ case HUMIDITY: error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_HM); break;
		case TEMP : error |= I2c_WriteByte (TRIG_T_MEASUREMENT_HM); break;
		//default: assert(0);
	}
	//-- wait until hold master is released --
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_R);
	//SCL=HIGH; // set SCL I/O port as input
	DDR_USI&=~(1<<PIN_USI_SCL);
	for(i=0; i<1000; i++) // wait until master hold is released or ;;;;; Son quatsch.... 1000 s *kopfschuettel*
	{ _delay_ms(1); // a timeout (~1s) is reached
		//if (SCL_CONF==1) break;
		if (PIN_USI&(1<<PIN_USI_SCL)) break;
	}
	//-- check for timeout --
	//Was wenn der SHT2x die leitung auf 0 laesst? Kurzschluss???
	if((PIN_USI&(1<<PIN_USI_SCL))==0) error |= TIME_OUT_ERROR; else DDR_USI|=(1<<PIN_USI_SCL);
	
	//-- read two data bytes and one checksum byte --
	*pMeasurand=((data[0] = I2c_ReadByte(ACK))>>8) & 0xFF;
	*pMeasurand|=0xFF & (data[1] = I2c_ReadByte(ACK));
//	pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
//	pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);

	checksum=I2c_ReadByte(NO_ACK);
	//-- verify checksum --
	error |= SHT2x_CheckCrc (data,2,checksum);
	I2c_StopCondition();
	return error;
}
//===========================================================================
uint8_t SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, int16_t *pMeasurand)
//===========================================================================
{
	uint8_t checksum; //checksum
	uint8_t data[2]; //data array for checksum verification
	uint8_t error=0; //error variable
	uint16_t i=0; //counting variable
	//-- write I2C sensor address and command --
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	switch(eSHT2xMeasureType)
	{ case HUMIDITY: error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_POLL); break;
		case TEMP : error |= I2c_WriteByte (TRIG_T_MEASUREMENT_POLL); break;
		//default: assert(0);
	}
	//-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
	do
	{ I2c_StartCondition();
		_delay_ms(200); //delay 10ms
		if(i++ >= 20) break;
	} while(I2c_WriteByte (I2C_ADR_R) == ACK_ERROR);
	if (i>=20) error |= TIME_OUT_ERROR;
	//-- read two data bytes and one checksum byte --
	data[0]=I2c_ReadByte(ACK);
	data[1]=I2c_ReadByte(ACK);
	*pMeasurand=(data[0]<<8)|data[1];
	
//	pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
//	pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);
	checksum=I2c_ReadByte(NO_ACK);
	//-- verify checksum --
	error |= SHT2x_CheckCrc (data,2,checksum);
	I2c_StopCondition();
	return error;
}
//===========================================================================
uint8_t SHT2x_SoftReset(void)
//===========================================================================
{
	uint8_t error=0; //error variable
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
	error |= I2c_WriteByte (SOFT_RESET); // Command
	I2c_StopCondition();
	_delay_ms(15); // wait till sensor has restarted
	return error;
}
//==============================================================================
float SHT2x_CalcRH(uint16_t u16sRH)
//==============================================================================
{
	double humidityRH; // variable for result
	u16sRH &= ~0x0003; // clear bits [1..0] (status bits)
	//-- calculate relative humidity [%RH] --
	humidityRH = -6.0 + 125.0/65536 * (double)u16sRH; // RH= -6 + 125 * SRH/2^16
	return humidityRH;
}
//==============================================================================
float SHT2x_CalcTemperatureC(uint16_t u16sT)
//==============================================================================
{
	double temperatureC; // variable for result
	u16sT &= ~0x0003; // clear bits [1..0] (status bits)
	//-- calculate temperature [°C] --
	temperatureC= -46.85 + 175.72/65536 *(double)u16sT; //T= -46.85 + 175.72 * ST/2^16
	return temperatureC;
}
//==============================================================================
uint8_t SHT2x_GetSerialNumber(uint8_t u8SerialNumber[])
//==============================================================================
{
	uint8_t error=0; //error variable
	//Read from memory location 1
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); //I2C address
	error |= I2c_WriteByte (0xFA); //Command for readout on-chip memory
	error |= I2c_WriteByte (0x0F); //on-chip memory address
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_R); //I2C address
	u8SerialNumber[5] = I2c_ReadByte(ACK); //Read SNB_3
	I2c_ReadByte(ACK); //Read CRC SNB_3 (CRC is not analyzed)
	u8SerialNumber[4] = I2c_ReadByte(ACK); //Read SNB_2
	I2c_ReadByte(ACK); //Read CRC SNB_2 (CRC is not analyzed)
	u8SerialNumber[3] = I2c_ReadByte(ACK); //Read SNB_1
	I2c_ReadByte(ACK); //Read CRC SNB_1 (CRC is not analyzed)
	u8SerialNumber[2] = I2c_ReadByte(ACK); //Read SNB_0
	I2c_ReadByte(NO_ACK); //Read CRC SNB_0 (CRC is not analyzed)
	I2c_StopCondition();
	//Read from memory location 2
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_W); //I2C address
	error |= I2c_WriteByte (0xFC); //Command for readout on-chip memory
	error |= I2c_WriteByte (0xC9); //on-chip memory address
	I2c_StartCondition();
	error |= I2c_WriteByte (I2C_ADR_R); //I2C address
	u8SerialNumber[1] = I2c_ReadByte(ACK); //Read SNC_1
	u8SerialNumber[0] = I2c_ReadByte(ACK); //Read SNC_0
	I2c_ReadByte(ACK); //Read CRC SNC0/1 (CRC is not analyzed)
	u8SerialNumber[7] = I2c_ReadByte(ACK); //Read SNA_1
	u8SerialNumber[6] = I2c_ReadByte(ACK); //Read SNA_0
	I2c_ReadByte(NO_ACK); //Read CRC SNA0/1 (CRC is not analyzed)
	I2c_StopCondition();
	return error;
}
*/