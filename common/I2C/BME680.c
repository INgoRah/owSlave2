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
#ifdef  __4MHZ__
#define F_CPU 4000000UL
#else
#define F_CPU 8000000UL
#endif
#include <avr/io.h>


/** Array Index to Field data mapping for Calibration Data*/
#define BME680_T2_LSB_REG	(1)
#define BME680_T2_MSB_REG	(2)
#define BME680_T3_REG		(3)
#define BME680_P1_LSB_REG	(5)
#define BME680_P1_MSB_REG	(6)
#define BME680_P2_LSB_REG	(7)
#define BME680_P2_MSB_REG	(8)
#define BME680_P3_REG		(9)
#define BME680_P4_LSB_REG	(11)
#define BME680_P4_MSB_REG	(12)
#define BME680_P5_LSB_REG	(13)
#define BME680_P5_MSB_REG	(14)
#define BME680_P7_REG		(15)
#define BME680_P6_REG		(16)
#define BME680_P8_LSB_REG	(19)
#define BME680_P8_MSB_REG	(20)
#define BME680_P9_LSB_REG	(21)
#define BME680_P9_MSB_REG	(22)
#define BME680_P10_REG		(23)
#define BME680_H2_MSB_REG	(25)
#define BME680_H2_LSB_REG	(26)
#define BME680_H1_LSB_REG	(26)
#define BME680_H1_MSB_REG	(27)
#define BME680_H3_REG		(28)
#define BME680_H4_REG		(29)
#define BME680_H5_REG		(30)
#define BME680_H6_REG		(31)
#define BME680_H7_REG		(32)
#define BME680_T1_LSB_REG	(33)
#define BME680_T1_MSB_REG	(34)
#define BME680_GH2_LSB_REG	(35)
#define BME680_GH2_MSB_REG	(36)
#define BME680_GH1_REG		(37)
#define BME680_GH3_REG	(38)


#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "TWI_Master.h"
#include "BME680.h"

#define WC 0b11101100
#define RC 0b11101101
typedef union {
	volatile uint8_t d[41];
	struct {
		uint8_t fr1;
		int16_t t2;
		int8_t t3;
		uint8_t fr2; //4
		uint16_t p1;
		int16_t p2; //7
		int8_t p3;
		uint8_t fr3; //10
		int16_t p4;
		int16_t p5;
		int8_t p7;
		int8_t p6;
		uint8_t fr4[2]; //17-18
		int16_t p8;
		int16_t p9;
		uint8_t p10;
		uint8_t fr5; //24
		uint16_t h2_; //25 +26
		uint8_t h1_;  //halb und halb 26+27
		int8_t h3;
		int8_t h4;
		int8_t h5;
		uint8_t h6;
		int8_t h7;//32
		uint16_t t1;
		int16_t gh2;
		int8_t gh1;
		int8_t gh3; //38
		int8_t fr6;// 39;
		int8_t fr7; //40;
		uint16_t h2; //Berechnung
		uint16_t h1; //Berechnung	
		int32_t t_fine; //Berechnung bei Temperaturmessung	
		uint8_t  res_heat_range;/**<resistance calculation*/
		int8_t  res_heat_val; /**<correction factor*/
		int8_t  range_switching_error;/**<range switching error*/
		int8_t ltemp; //letzte Temperatur
	};
} calib_t;

volatile calib_t calib;


#define BME680_CALIB_I2C_ADDR_1				(0x89)
#define BME680_CALIB_I2C_ADDR_2				(0xE1)
#define BME680_PAGE0_I2C_ID_REG				(0xD0)
#define BME680_CALIB_DATA_LENGTH_GAS			(25)
#define BME680_CALIB_DATA_LENGTH			(16)


#define BME680_MAX_HUMIDITY_VALUE		(102400)
#define BME680_MIN_HUMIDITY_VALUE		(0)


//ME680_CALIB_I2C_ADDR_1,
//						a_data_u8,
//						BME680_CALIB_DATA_LENGTH_GAS);
				/* read the humidity and gas
				calibration data*/
/*				com_status = (enum bme680_return_type)
					     bme680->bme680_bus_read(
					     bme680->dev_addr,
					     BME680_CALIB_I2C_ADDR_2,
					    (a_data_u8 +
					    BME680_CALIB_DATA_LENGTH_GAS),
					    BME680_CALIB_DATA_LENGTH);



						*/


void setup_read(uint8_t addr) {
	I2c_StartCondition();
	I2c_WriteByte(WC);
	I2c_WriteByte(addr);  //Ctrl hum
	I2c_StartCondition();
	I2c_WriteByte (RC);
	
}

void setup_write(uint8_t addr) {
	I2c_StartCondition();
	I2c_WriteByte(WC);
	I2c_WriteByte(addr);  //Ctrl hum
}


uint8_t readone(uint8_t addr) {
	setup_read(addr);
	uint8_t b=I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	return b;
}

void writeone(uint8_t addr,uint8_t b) {
	setup_write(addr);
	I2c_WriteByte(b);  //Ctrl hum
	I2c_StopCondition();
}

int8_t initBME680() {
	uint8_t b1=readone(0xD0);
	setup_read(BME680_CALIB_I2C_ADDR_1);
	for(uint8_t i=0;i<BME680_CALIB_DATA_LENGTH_GAS-1;i++) {
		calib.d[i]=I2c_ReadByte(ACK);
	}
	calib.d[BME680_CALIB_DATA_LENGTH_GAS-1]=I2c_ReadByte(NO_ACK);
	I2c_StopCondition();

	setup_read(BME680_CALIB_I2C_ADDR_2);
	for(uint8_t i=BME680_CALIB_DATA_LENGTH_GAS;i<BME680_CALIB_DATA_LENGTH_GAS+BME680_CALIB_DATA_LENGTH-1;i++) {
		calib.d[i]=I2c_ReadByte(ACK);
	}
	calib.d[BME680_CALIB_DATA_LENGTH_GAS+BME680_CALIB_DATA_LENGTH-1]=I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	calib.res_heat_range=(readone(0x02)&0x30)>>4;
	calib.res_heat_val=readone(0);
	calib.range_switching_error=(readone(0x04)& 0xF0)>>4;


    calib.h1 = (uint16_t)(((((uint16_t)calib.d[ BME680_H1_MSB_REG]))
		<< 4) | (calib.d[ BME680_H1_LSB_REG] &0x0F));
	calib.h2 = (uint16_t)(((((uint16_t)calib.d[ BME680_H2_MSB_REG]))
		<< 4) | ((calib.d[ BME680_H2_LSB_REG]) >> 4));

	
	/*
	
	I2c_StartCondition();
	I2c_WriteByte(WC);
	I2c_WriteByte(0x72);  //Ctrl hum
	I2c_WriteByte(0x01); //1x oversembling hum
	I2c_WriteByte(0x74);  //Ctrl hum
	I2c_WriteByte(0b01010101); //2x oversembling t - 16x oversemmping p - mode cont
	I2c_StopCondition();	

	*/
	calib.ltemp=25;

	return b1==0x61;

}


const float lookup_k1_range[16] PROGMEM = {
	1, 1, 1, 1, 1,0.99, 1, 0.992,
1, 1, 0.998, 0.995, 1, 0.99, 1, 1};
const float lookup_k2_range[16] PROGMEM = {
	8e6, 4e6, 2e6, 1e6,499500.4995, 248262.1648, 125000, 63004.03226,
31281.28128, 15625, 7812.5, 3906.25,1953.125,976.5625, 488.28125, 244.140625};

const float lookup_k1_range1[16]  = {
	1, 1, 1, 1, 1,0.99, 1, 0.992,
1, 1, 0.998, 0.995, 1, 0.99, 1, 1};
const float lookup_k2_range1[16]  = {
	8e6, 4e6, 2e6, 1e6,499500.4995, 248262.1648, 125000, 63004.03226,
31281.28128, 15625, 7812.5, 3906.25,1953.125,976.5625, 488.28125, 244.140625};


double bme680_compensate_gas_double(uint16_t gas_adc_u16, uint8_t gas_range_u8)
{
	double gas_res_d = 0;

	
	
	int8_t range_switching_error_val = 0;

	double var1 = 0;
	float a1= pgm_read_float(&(lookup_k1_range[gas_range_u8]));
	float a2= pgm_read_float(&(lookup_k2_range[gas_range_u8]));
	//float a1=lookup_k1_range1[gas_range_u8];
	//float a2=lookup_k2_range1[gas_range_u8];

	range_switching_error_val =	calib.range_switching_error;


	var1 = (1340.0 + (5.0 * range_switching_error_val))*a1;

	gas_res_d = var1*a2/(gas_adc_u16-512.0+var1);
	return gas_res_d;
}


void readBMP680(int16_t *T,uint16_t *H,uint32_t *P,uint16_t *G){


	
	int32_t var1 ;
	int32_t var2 ;
	int32_t var3 ;
	int32_t var4 ;
	int32_t var5 ;
	int32_t res_heat_x100 = 0;
	uint8_t res_heat = 0;
	uint16_t heater_temp_u16=350;
	int16_t ambient_temp_s16=calib.ltemp;
	if ((heater_temp_u16 >= 200) && (heater_temp_u16 <= 400)) {
		var1 = (((int32_t)ambient_temp_s16 *
		calib.gh3) / 10) << 8;
		var2 = (calib.gh1 + 784) *
		(((((calib.gh2 + 154009) *
		heater_temp_u16 * 5) / 100) + 3276800) / 10);
		var3 = var1 + (var2 >> 1);
		var4 = (var3 / (calib.res_heat_range + 4));

		var5 = (131 * calib.res_heat_val) + 65536;

		res_heat_x100 = (int32_t)(((var4 / var5) - 250) * 34);
		res_heat = (uint8_t) ((res_heat_x100 + 50) / 100);
	}
	uint16_t duration=100;
	uint8_t factor = 0;

	while ((duration) > 0x3F) {
		(duration) = (duration) >> 2;
		factor += 1;
	}
	(duration) = (duration) + (factor * 64);

	//I2c_WriteByte(0x74);  //Ctrl hum
	//I2c_WriteByte(0b01010101); //2x oversembling t - 16x oversemmping p - mode cont
	// [71] <- 10;  [72] <- 04;  [73] <- 0C;  [74] <- 91;  [75] <- 00;
	// [70] <- 00     [71] <- 10;  [72] <- 04;  [73] <- 0C;  [74] <- 91;  [75] <- 00;
	setup_write(0x70);
	I2c_WriteByte(0x00);
	I2c_WriteByte(0x71);
	I2c_WriteByte(0x10);
	I2c_WriteByte(0x72);
	I2c_WriteByte(0x04);
	I2c_WriteByte(0x73);
	I2c_WriteByte(0x0C);
	I2c_WriteByte(0x74);
	I2c_WriteByte(0x90);
	I2c_WriteByte(0x75);
	I2c_WriteByte(0x00);

	I2c_WriteByte(0x5A);
	I2c_WriteByte(res_heat);
	I2c_WriteByte(0x64);
	I2c_WriteByte(duration);
	I2c_StopCondition();


	writeone(0x74,0x91);
	_delay_ms(1000);
	
	uint8_t bx=0x91;
	while ((bx&0x01)==0x01) {
		bx=readone(0x74);
		_delay_ms(5);
	}

	//volatile uint8_t rs=readone(0x2B);
	uint32_t Th,Hh,Ph;
	setup_read(0x1F);
	Ph=I2c_ReadByte(ACK);Ph=Ph<<8;
	Ph|=I2c_ReadByte(ACK);Ph=Ph<<4;
	Ph|=I2c_ReadByte(ACK)>>4;
	Th=I2c_ReadByte(ACK);Th=Th<<8;
	Th|=I2c_ReadByte(ACK);Th=Th<<4;
	Th|=I2c_ReadByte(ACK)>>4;	
	Hh=I2c_ReadByte(ACK);Hh=Hh<<8;
	Hh|=I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	setup_read(0x2A);
	volatile uint8_t g1=I2c_ReadByte(ACK);
	volatile uint8_t g2=I2c_ReadByte(NO_ACK);
	I2c_StopCondition();
	*G=(((uint16_t)g1)<<2)|(g2>>6);
	*P=*G;
	
	*G= bme680_compensate_gas_double(*G,g2&0xF)/10.0;

	int32_t temp_comp = 0;

	var1 = ((int32_t)Th >> 3) -
	((int32_t)calib.t1 << 1);
	var2 = (var1 * (int32_t)calib.t2) >> 11;
	var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) *
	((int32_t)calib.t3 << 4)) >> 14;
	calib.t_fine = var2 + var3;
	temp_comp = ((calib.t_fine * 5) + 128) >> 8;

	int32_t temp_scaled = 0;
	int32_t var6	= 0;
	int32_t humidity_comp = 0;

	temp_scaled = (((int32_t)calib.t_fine * 5) + 128) >> 8;
	var1 = (int32_t)Hh -
		((int32_t)((int32_t)calib.h1 << 4)) -
		(((temp_scaled * (int32_t)calib.h3) /
		((int32_t)100)) >> 1);

	var2 = ((int32_t)calib.h2 *
		(((temp_scaled * (int32_t)calib.h4) /
		((int32_t)100)) + (((temp_scaled *
		((temp_scaled * (int32_t)calib.h5) /
		((int32_t)100))) >> 6) / ((int32_t)100)) + (int32_t)(1 << 14))) >> 10;

	var3 = var1 * var2;

	var4 = ((((int32_t)calib.h6) << 7) +
		((temp_scaled * (int32_t)calib.h7) /
		((int32_t)100))) >> 4;

	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;

	humidity_comp = (var3 + var6) >> 12;
	if (humidity_comp > BME680_MAX_HUMIDITY_VALUE)
		humidity_comp = BME680_MAX_HUMIDITY_VALUE;
		else if (humidity_comp < BME680_MIN_HUMIDITY_VALUE)
		humidity_comp = BME680_MIN_HUMIDITY_VALUE;

    int32_t pressure_comp = 0;//int -> 5684

    var1 = (((int32_t)calib.t_fine) >> 1) - (int32_t)64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
    (int32_t)calib.p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)calib.p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)calib.p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
    ((int32_t)calib.p3 << 5)) >> 3) +
    (((int32_t)calib.p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = (((int32_t)32768 + var1) * (int32_t)calib.p1) >> 15;
    pressure_comp = (int32_t)1048576 - Ph;
    pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
    var4 = ((int32_t)1 << 31);
    if (pressure_comp >= var4)
    pressure_comp = ((pressure_comp / (uint32_t)var1) << 1);
    else
    pressure_comp = ((pressure_comp << 1) / (uint32_t)var1);
    var1 = ((int32_t)calib.p9 * (int32_t)(((pressure_comp >> 3) *
    (pressure_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(pressure_comp >> 2) *
    (int32_t)calib.p8) >> 13;
    var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
    (int32_t)(pressure_comp >> 8) *
    (int32_t)calib.p10) >> 17;

    pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 +
    ((int32_t)calib.p7 << 7)) >> 4);




	
	*T=(int16_t)temp_comp;
	calib.ltemp=temp_comp/100;
	//*P=pressure_comp;
	*H=(uint16_t)(humidity_comp/10);
	//*P=rs;
	*T=g1;
	*H=g2;
	//*P=

}

