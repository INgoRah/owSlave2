
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
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include "../common/I2C/USI_TWI_Master.h"
#include "../common/I2C/SHT2xV2.h"
#include "../common/owSlave_tools.h"

OWST_EXTERN_VARS

//#define K_Type
#define J_Type

#ifdef K_Type
//const float k_rs[54] PROGMEM ={0.000000,24.125000,48.585366,72.731707,96.829268,121.097561,145.700000,170.600000,195.650000,220.625000,245.365854,269.853659,294.119048,318.195122,342.166667,366.000000,389.761905,413.428571,437.023810,460.558140,484.047619,507.511628,530.976190,554.418605,577.883721,601.395349,624.952381,648.571429,672.285714,696.073171,719.976190,744.000000,768.146341,792.439024,816.853659,841.414634,866.125000,890.975000,916.000000,941.179487,966.525000,992.025641,1017.717949,1043.589744,1069.657895,1095.945946,1122.432432,1149.184211,1176.189189,1203.472222,1231.083333,1259.000000,1287.285714,1315.941176};
const float k_rs[61] PROGMEM ={ -212.538462, -166.260870, -124.892857, -97.562500, -66.888889, -34.157895, 0.000000, 24.125000, 48.585366, 72.731707, 96.829268, 121.097561, 145.700000, 170.600000, 195.650000, 220.625000, 245.365854, 269.853659, 294.119048, 318.192771, 342.166667, 366.000000, 389.761905, 413.428571, 437.023810, 460.558140, 484.047619, 507.511628, 530.976190, 554.418605, 577.883721, 601.395349, 624.952381, 648.571429, 672.285714, 696.073171, 719.976190, 744.000000, 768.146341, 792.439024, 816.853659, 841.414634, 866.125000, 890.975000, 916.000000, 941.179487, 966.525000, 992.025641, 1017.717949, 1043.589744, 1069.657895, 1095.945946, 1122.432432, 1149.173333, 1176.189189, 1203.472222, 1231.083333, 1259.000000, 1287.285714, 1315.941176, 1344.941176};
#define k_ofs 6
#endif
#ifdef J_Type
#define k_ofs 0
//J-Type
const float k_rs[70] PROGMEM ={0, 18.302913, 34.830476, 50.783019, 70.653704, 90.505455, 110.341818, 130.165455, 149.163636, 160.791071, 180.596364, 200.398214, 220.200000, 240.000000, 250.882883, 270.603636, 290.409091, 310.216364, 330.025455, 342.472727, 360.649091, 380.461818, 400.275000, 420.087273, 435.275676, 450.703636, 470.503636, 490.298214, 510.082456, 523.486726, 540.621053, 560.370175, 580.105172, 591.979487, 610.527119, 630.213559, 644.601653, 660.534426, 680.168852, 690.787097, 710.391935, 729.123810, 740.559375, 760.126562, 770.684615, 790.235385, 800.782812, 820.331250, 834.681250, 850.446032, 870.017460, 880.600000, 900.196774, 911.099187, 930.432787, 950.073333, 960.728333, 980.396667, 1000.078333, 1010.772881, 1030.475862, 1050.187931, 1065.717241, 1080.631034, 1100.358621, 1120.089655, 1131.840000,1150.556897, 1170.294737, 1190.035088};
#endif

double gettemp_rs(double V) {
	uint8_t iv=(uint8_t)(V+k_ofs);
	float t0=pgm_read_float(&(k_rs[iv]));
	float t1=pgm_read_float(&(k_rs[iv+1]));
	return t0+(t1-t0)/1*((V+k_ofs)-iv);
}





uint8_t owid1[8]={0x28, 0xA6, 0xD9, 0x84, 0x00, 0x00, 0x03, 0x68};/**/
uint8_t owid2[8]={0x26, 0xA7, 0xD9, 0x84, 0x00, 0x00, 0x03, 0x20};/**/
uint8_t config_info1[26]={0x01,0x02 ,0x00,0x00, 0x00,0x00, 0x00,0x00, 0x02,6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
uint8_t config_info2[26]={0x01,0x06, 0x05,0x08, 0x04,0x07, 0x00,0x00, 0x02,7,0x00,7,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
	
OWST_WDT_ISR
OWST_TESTSW


typedef union {
	volatile uint8_t bytes[8];
	struct {
		uint16_t temp;  //0
		uint8_t TH;  //2
		uint8_t TL;  //3
		uint8_t config;  //4
		uint8_t rrFF; //5
		uint8_t rr00; //6
		uint8_t rr10; //7
	};
} pack1_t;
volatile pack1_t pack1;



typedef union {
	#if  defined(__AVR_ATtiny25__)
	volatile uint8_t bytes[16];
	#else
	volatile uint8_t bytes[64];
	#endif
	struct {
		uint8_t status;  //1
		int16_t temp;  //2
		uint16_t voltage;  //4
		uint16_t current;  //6
		uint8_t threshold; //8
		
		uint8_t page1[8]; //9
		#if  defined(__AVR_ATtiny25__)
		#else
		uint8_t page2[8]; //17
		uint8_t page3[8]; //25
		uint8_t page4[8];  //33
		uint8_t page5[8];  //41
		uint8_t page6[8];  //49
		uint8_t page7[8];  //57
		//uint8_t crc;  //65
		#endif
	};
} pack2_t;
volatile pack2_t pack2;


volatile double V,ktemp;

double ADmess() {
	ADMUX=0b10101101;  //PA2 + //PA1 - 
	ADCSRA|=(1<<ADSC);
	while ((ADCSRA&(1<<ADSC)));
	if (ADC==0) {
		ADMUX=0b10001101; //PA2 - //PA1 +
		ADCSRA|=(1<<ADSC);
		while ((ADCSRA&(1<<ADSC)));
		return -(double)ADC;
	}
	return (double)ADC;
}

volatile int16_t DS2438_2_TEMP;
volatile uint16_t DS2438_2_VAD;
volatile uint16_t DS2438_2_VDD=0x01F4;



double temperatureC,humidityRH;
double TC;
double l;

int main(void){
	OWST_INIT_ALL_ON
 	pack1.temp=0x0550;
	pack1.config=0x7F;
	pack1.TH=75;
	pack1.TL=70;
	pack1.rrFF=0xFF;
	pack1.rr00=0;
	pack1.rr10=0x10;
	PORTA=0xFF-(1<<PINA1)-(1<<PINA2);
	OWINIT();

	OWST_EN_PULLUP
	OWST_WDR_CONFIG1	

	if (testSW()) {
		config_info2[5]=12;
		}else{
		config_info2[5]=7;
	}
	USI_TWI_Master_Initialise();
	initSHT2x();
		
	gcontrol=1;
	for(uint8_t i=0;i<8;i++) pack2.page6[i]=owid1[i];
	pack2.page3[0]=0xF1;
	sei();
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	//DDRB|=(1<<PINB1);

	double ares[16],sum;
	uint8_t par=0;
	ares[0]=0;//ADmess();
	for (par=1;par<16;par++) {
		ares[par]=ares[0];
	}
	par=0;
    while(1)   {
		if ((gcontrol==2)||(gcontrol==3)) {
			wdcounter=1;
			gcontrol=0;
			
		}
		
		if (wdcounter>0) {
			if (par==0) {
				getSHT2xHumTemp(&temperatureC,&humidityRH);
				double RH=humidityRH*10.0;
				 TC =temperatureC *10.0;

				if (testSW()) {
					DS2438_2_VAD=RH;
					DS2438_2_TEMP=TC*25.6;
		
					config_info2[5]=12;
					
					}else{
					
					double hhum=(1.0546-0.000216*TC)*(RH);
					DS2438_2_VAD=0.31*hhum +80;
					DS2438_2_TEMP=TC*25.6;
					config_info2[5]=7;
				}
			}
			DS2438_2_VDD=0x01F4;
			ares[par]=ADmess();
			par++;
			if (par>15) par=0;
			wdcounter=0;
		}
	
		if ((gcontrol&1)==1) {
			gcontrol=0;
			sum=0;
			for(uint8_t i=0;i<16;i++) {
				sum+=ares[i];
			}
			V=sum/20.0/1024.0*1.18*1000.0/16.0;//Spannung in mV
			//V=sum/20.0/1024.0*1.01*1000.0/16.0;
			int16_t htemp;
			ktemp=gettemp_rs(V);
			//htemp=(ktemp*16)/10;
			htemp=(ktemp*16+TC*1.6)/10;

			int16_t w=htemp;
			int8_t t8=w>>4;
			int8_t af=0;
			if (t8>pack1.TH) af=1;
			if (t8<=pack1.TL) af=1; 
			cli();
			pack1.temp=w;
			//pack.temp++;
			alarmflag=af;
			sei();			
			EXTERN_SLEEP();		
		}
		

		
#if  defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)  || defined(__AVR_ATtiny85__)
			if (((TIMSK & (1<<TOIE0))==0)&& (mode==0))
#endif			
#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) ||defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
			if (((TIMSK0 & (1<<TOIE0))==0)&& (mode==0))
#endif
			  {

			MCUCR|=(1<<SE)|(1<<SM1);
			MCUCR&=~(1<<ISC01);
		} else {
			MCUCR|=(1<<SE);
			MCUCR&=~(1<<SM1);
		}
		asm("SLEEP");
   }


}