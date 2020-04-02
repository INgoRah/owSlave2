#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "TWI_Master.h"

void TSL256x_init() {
	I2c_StartCondition();
	I2c_WriteByte(0b01010010);  // 0x0101001 1001001 0111001
	I2c_WriteByte(0x80);
	I2c_WriteByte(0x03);
	I2c_StopCondition();
	I2c_StartCondition();
	I2c_WriteByte(0b01010010);  // 0x0101001 1001001 0111001
	I2c_WriteByte(0x81);
	I2c_WriteByte(0x12);
	I2c_StopCondition();
	_delay_ms(500);
}

void TSL256x_setup(uint8_t conf) {
	I2c_StartCondition();
	I2c_WriteByte(0b01010010);  // 0x0101001 1001001 0111001
	I2c_WriteByte(0x81);
	I2c_WriteByte(conf);
	I2c_StopCondition();
}

uint16_t TSL256x_Ch(uint8_t ch){
	uint16_t res;
	I2c_StartCondition();
	I2c_WriteByte(0b01010010);  // 0x0101001 1001001 0111001
	//I2c_WriteByte(0xAD);
	I2c_WriteByte(0xAC+(ch<<1));
	I2c_StopCondition();
	I2c_StartCondition();
	I2c_WriteByte(0b01010011); 
	res=I2c_ReadByte(ACK);
	res|=((uint16_t)I2c_ReadByte(NO_ACK))<<8;
	I2c_StopCondition();
	return res;
}

#define LUX_SCALE 14 // scale by 2^14
#define RATIO_SCALE 9 // scale ratio by 2^9
//???????????????????????????????????????????????????
// Integration time scaling factors
//???????????????????????????????????????????????????
#define CH_SCALE 10 // scale channel values by 2^10
#define CHSCALE_TINT0 0x7517 // 322/11 * 2^CH_SCALE
#define CHSCALE_TINT1 0x0fe7 // 322/81 * 2^CH_SCALE

#define K1T 0x0040 // 0.125 * 2^RATIO_SCALE
#define B1T 0x01f2 // 0.0304 * 2^LUX_SCALE
#define M1T 0x01be // 0.0272 * 2^LUX_SCALE
#define K2T 0x0080 // 0.250 * 2^RATIO_SCALE
#define B2T 0x0214 // 0.0325 * 2^LUX_SCALE
#define M2T 0x02d1 // 0.0440 * 2^LUX_SCALE
#define K3T 0x00c0 // 0.375 * 2^RATIO_SCALE
#define B3T 0x023f // 0.0351 * 2^LUX_SCALE
#define M3T 0x037b // 0.0544 * 2^LUX_SCALE
#define K4T 0x0100 // 0.50 * 2^RATIO_SCALE
#define B4T 0x0270 // 0.0381 * 2^LUX_SCALE
#define M4T 0x03fe // 0.0624 * 2^LUX_SCALE
#define K5T 0x0138 // 0.61 * 2^RATIO_SCALE
#define B5T 0x016f // 0.0224 * 2^LUX_SCALE
#define M5T 0x01fc // 0.0310 * 2^LUX_SCALE
#define K6T 0x019a // 0.80 * 2^RATIO_SCALE
#define B6T 0x00d2 // 0.0128 * 2^LUX_SCALE
#define M6T 0x00fb // 0.0153 * 2^LUX_SCALE
#define K7T 0x029a // 1.3 * 2^RATIO_SCALE
#define B7T 0x0018 // 0.00146 * 2^LUX_SCALE
#define M7T 0x0012 // 0.00112 * 2^LUX_SCALE
#define K8T 0x029a // 1.3 * 2^RATIO_SCALE
#define B8T 0x0000 // 0.000 * 2^LUX_SCALE
#define M8T 0x0000 // 0.000 * 2^LUX_SCALE

uint32_t CalculateLux(uint8_t iGain, uint8_t tInt, uint16_t ch0, uint16_t ch1)
{
	//????????????????????????????????????????????????????????????????????????
	// first, scale the channel values depending on the gain and integration time
	// 16X, 402mS is nominal.
	// scale if integration time is NOT 402 msec
	uint32_t chScale;
	uint32_t channel1;
	uint32_t channel0;
	switch (tInt)
	{
		case 0: // 13.7 msec
		chScale = CHSCALE_TINT0;
		break;
		case 1: // 101 msec
		chScale = CHSCALE_TINT1;
		break;
		default: // assume no scaling
		chScale = (1 << CH_SCALE);
		break;
	}
	// scale if gain is NOT 16X
	if (!iGain) chScale = chScale << 4; // scale 1X to 16X
	// scale the channel values
	channel0 = (ch0 * chScale) >> CH_SCALE;
	channel1 = (ch1 * chScale) >> CH_SCALE;
	//????????????????????????????????????????????????????????????????????????
	// find the ratio of the channel values (Channel1/Channel0)
	// protect against divide by zero
	uint32_t ratio1 = 0;
	if (channel0 != 0) ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;
	// round the ratio value
	uint32_t ratio = (ratio1 + 1) >> 1;
	// is ratio <= eachBreak ?
	uint16_t b, m;
		if ((ratio >= 0) && (ratio <= K1T))
		{b=B1T; m=M1T;}
		else if (ratio <= K2T)
		{b=B2T; m=M2T;}
		else if (ratio <= K3T)
		{b=B3T; m=M3T;}
		else if (ratio <= K4T)
		{b=B4T; m=M4T;}
		else if (ratio <= K5T)
		{b=B5T; m=M5T;}
		else if (ratio <= K6T)
		{b=B6T; m=M6T;}
		else if (ratio <= K7T)
		{b=B7T; m=M7T;}
		else if (ratio > K8T)
		{b=B8T; m=M8T;}
	uint32_t temp;
	temp = ((channel0 * b) - (channel1 * m));
	// do not allow negative lux value
	if (temp < 0) temp = 0;
	// round lsb (2^(LUX_SCALE?1))
	temp += (1 << (LUX_SCALE-1));
	// strip off fractional portion
	uint32_t lux = temp ;//>> LUX_SCALE;
	return(lux);
}




	

