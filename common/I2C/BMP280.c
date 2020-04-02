#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "TWI_Master.h"

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;




short bmp280ReadShort(unsigned char address)
{
	char msb, lsb;
	short data;
	I2c_StartCondition();
	I2c_WriteByte(0xEC);
	I2c_WriteByte(address);
	I2c_StopCondition();
	I2c_StartCondition();
	I2c_WriteByte(0xED);
	lsb=I2c_ReadByte(ACK);
	msb=I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	

	
	data = msb << 8;
	data |= lsb;
	
	return data;
}

void bmp280Init(void) {
	I2c_StartCondition();
	I2c_WriteByte(0xEC);
	I2c_WriteByte(0xF4);
	I2c_WriteByte(0b01010111); //2x Temp over 16x Press over   Normal mode
	I2c_WriteByte(0b11100000);  //4s time / no IIfilter
	I2c_StopCondition();
	dig_T1=bmp280ReadShort(0x88);
	dig_T2=bmp280ReadShort(0x8A);
	dig_T3=bmp280ReadShort(0x8C);
	dig_P1=bmp280ReadShort(0x8E);
	dig_P2=bmp280ReadShort(0x90);
	dig_P3=bmp280ReadShort(0x92);
	dig_P4=bmp280ReadShort(0x94);
	dig_P5=bmp280ReadShort(0x96);
	dig_P6=bmp280ReadShort(0x98);
	dig_P7=bmp280ReadShort(0x9A);
	dig_P8=bmp280ReadShort(0x9C);
	dig_P9=bmp280ReadShort(0x9E);
	
	
	
}
int32_t bmp280ReadTemp(void) {
	uint8_t msb, lsb,xlsb;
	volatile int32_t data;
	I2c_StartCondition();
	I2c_WriteByte(0xEC);
	I2c_WriteByte(0xFA);
	I2c_StopCondition();
	I2c_StartCondition();
	I2c_WriteByte(0xED);
	msb=I2c_ReadByte(ACK);
	lsb=I2c_ReadByte(ACK);
	xlsb=I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
		
	data = (int32_t)msb << 12;
	data |= (int16_t)lsb<<4;
	data|=xlsb>>4;
		
		return data;

	
}
int32_t bmp280ReadPressure(uint8_t oss) {
		uint8_t msb, lsb,xlsb;
		volatile int32_t data;
		I2c_StartCondition();
		I2c_WriteByte(0xEC);
		I2c_WriteByte(0xF7);
		I2c_StopCondition();
		I2c_StartCondition();
		I2c_WriteByte(0xED);
		msb=I2c_ReadByte(ACK);
		lsb=I2c_ReadByte(ACK);
		xlsb=I2c_ReadByte(NO_ACK);
		I2c_StopCondition();
		
		data = (int32_t)msb << 12;
		data |= (int16_t)lsb<<4;
		data|=xlsb>>4;
		
		return data;


}

int32_t bmp280ReadPressure_simple() {
		uint8_t msb, lsb,xlsb;
		volatile int32_t data;
		I2c_StartCondition();
		I2c_WriteByte(0xEC);
		I2c_WriteByte(0xF7);
		I2c_StopCondition();
		I2c_StartCondition();
		I2c_WriteByte(0xED);
		msb=I2c_ReadByte(ACK);
		lsb=I2c_ReadByte(ACK);
		xlsb=I2c_ReadByte(NO_ACK);
		I2c_StopCondition();
		
		data = (int32_t)msb << 12;
		data |= (int16_t)lsb<<4;
		data|=xlsb>>4;
		
		return data;


}
#define BMP280_S32_t int32_t
// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
// t_fine carries fine temperature as global value
BMP280_S32_t t_fine;
double bmp280_compensate_T_double(BMP280_S32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (BMP280_S32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}
// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double bmp280_compensate_P_double(BMP280_S32_t adc_P)
{
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}

void bmp280ConvertInt_df(int32_t * temperature,uint32_t *pressure,uint8_t oss) {
		//bmp280Init();
		double var1, var2, T;
		int32_t adc_T=bmp280ReadTemp();
		var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
		var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
		(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
		t_fine = (BMP280_S32_t)(var1 + var2);
		T = (var1 + var2) / 5120.0;

		int32_t adc_P= bmp280ReadPressure(oss);
		double  p;
		var1 = ((double)t_fine/2.0) - 64000.0;
		var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
		var2 = var2 + var1 * ((double)dig_P5) * 2.0;
		var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
		var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
		var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
		if (var1 == 0.0)
		{
			return ; // avoid exception caused by division by zero
		}
		p = 1048576.0 - (double)adc_P;
		p = (p - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)dig_P9) * p * p / 2147483648.0;
		var2 = p * ((double)dig_P8) / 32768.0;
		p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
		*temperature=(int32_t) (T*100.0);
		*pressure=(uint32_t) p;
		return;
}



void bmp280ConvertInt(int32_t * temperature,uint32_t *pressure,uint8_t oss) {
	int32_t var1,var2,T,t_fine;
	int32_t adc_T=bmp280ReadTemp();
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = ((((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - (int32_t)dig_T1))) >> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	*temperature=T;
	int32_t adc_P= bmp280ReadPressure(oss);
	int32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
	if (var1 == 0) {
		return ; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000){
		p = (p << 1) / ((uint32_t)var1);
		}else{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));

	*pressure=(uint32_t)(p);

}

void bmp280Convert(double * temperature, double * pressure,uint8_t oss) {

	uint32_t p;
	int32_t T;
	bmp280ConvertInt(&T,&p,oss);
	*temperature=T/100.0;
	*pressure=p/100.0;
}
		
void bmp280ConvertIntP(int32_t temp256,uint32_t *pressure) {
	int32_t var1,var2,t_fine;
	//T = (t_fine * 5 + 128) >> 8;
	t_fine=((temp256)-128)/5;
	int32_t adc_P= bmp280ReadPressure_simple();
	uint32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
	if (var1 == 0) {
		return ; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000){
		p = (p << 1) / ((uint32_t)var1);
		}else{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	*pressure=p;

}		
		
void bmp280ConvertIntP1(uint32_t *pressure) {
	int32_t var1,var2,t_fine;
	
	
	int32_t adc_T=bmp280ReadTemp();
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = ((((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - (int32_t)dig_T1))) >> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	//T = (t_fine * 5 + 128) >> 8;
	int32_t adc_P= bmp280ReadPressure_simple();
	uint32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
	if (var1 == 0) {
		return ; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000){
		p = (p << 1) / ((uint32_t)var1);
		}else{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	*pressure=p;

}
