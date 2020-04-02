
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
#include "../common/I2C/BMP280.h"
#include "../common/I2C/MAX44009.h"
#include "../common/owSlave_tools.h"
#include "../common/calibr.h"

OWST_EXTERN_VARS

uint8_t owid[8]={0x20, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x02, 0x5D};/**/
uint8_t config_info[26]={0x01,14,0x04,0x08, 0x03,15, 0x02,16,0x02,7,7,15,14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

#if (owid>128) 
#error "Variable not correct"
#endif

OWST_WDT_ISR


typedef union {
	volatile uint8_t bytes[0x20];
	struct {
		//Page0
		uint16_t A;  //0
		uint16_t B;  //2
		uint16_t C;  //4
		uint16_t D;  //6
		//Page1
		uint8_t CSA1;
		uint8_t CSA2;
		uint8_t CSB1;
		uint8_t CSB2;
		uint8_t CSC1;
		uint8_t CSC2;
		uint8_t CSD1;
		uint8_t CSD2;
		//Page2
		uint8_t LA;
		uint8_t HA;
		uint8_t LB;
		uint8_t HB;
		uint8_t LC;
		uint8_t HC;
		uint8_t LD;
		uint8_t HD;
		//Page3
		uint8_t FC1;
		uint8_t FC2;
		uint8_t FC3;
		uint8_t FC4;
		uint8_t VCCP;
		uint8_t FC5;
		uint8_t FC6;
		uint8_t FC7;
		uint8_t convc1;
		uint8_t convc2;
		
		
	};
} pack_t;
volatile pack_t pack;





 double temperatureC,humidityRH;
 double l;
uint32_t P;
int32_t t;
uint8_t max_adr=0;



int main(void){
	OWST_INIT_USI_ON
	pack.A=0;
	pack.B=0;
	pack.C=0;
	pack.D=0;
	pack.CSA1=0x08;
	pack.CSA2=0x8C;
	pack.CSB1=0x08;
	pack.CSB2=0x8C;
	pack.CSC1=0x08;
	pack.CSC2=0x8C;
	pack.CSD1=0x08;
	pack.CSD2=0x8C;
	pack.HA=0xFF;
	pack.LA=0x00;
	pack.HB=0xFF;
	pack.LB=0x00;
	pack.HC=0xFF;
	pack.LC=0x00;
	pack.HD=0xFF;
	pack.LD=0x00;
	pack.VCCP=0;

	OWINIT();

	OWST_WDR_CONFIG8
	OWST_EN_PULLUP
	
	gcontrol=1;
	
	initSHT2x();
	_delay_ms(10);
	bmp280Init();
	if (checkMAX44009(0)) max_adr=0; else max_adr=1 ;
	
	sei();
	
	//DDRB|=(1<<PINB1);

    while(1)   {
	
	if (wdcounter>3) {	
		getSHT2xHumTemp(&temperatureC,&humidityRH);
		humidityRH=calibr_hum(temperatureC,-0.2,humidityRH);
		temperatureC =temperatureC -0.2;
		bmp280ConvertInt(&t,&P,1);
		l=MAX44009getlux(max_adr);
		if (l<0.030) l=0.030; //Darf nicht 0 sein. minimum -35°C Sensor minimum 0.045
		//double l=1000;
		l=(log(l)*1000)+32767.0;
		wdcounter=0;
	}
	

		
		


		if (gcontrol) {
			//PORTB|=(1<<PINB1);
			uint8_t bb=1;
			uint8_t bb1=1;
			for(uint8_t i=0;i<4;i++){
				if (pack.convc1&bb1) {
					if (pack.convc2&(bb)) {pack.bytes[i*2]=0;pack.bytes[i*2+1]=0;}
					bb=bb<<1;
					if (pack.convc2&(bb)) {pack.bytes[i*2]=0xFF;pack.bytes[i*2+1]=0xFF;}
					bb=bb<<1;
				} else bb=bb<<2;
				bb1=bb1<<1;				
			}
			//CHanel A
			if (pack.convc1&1) {
				/*if (pack.CSA2&0x01)	ADMUX=0; else ADMUX=0x80;
				_delay_us(100);
				ADCSRA|=(1<<ADSC);
				while ((ADCSRA&(1<<ADSC)));
				cli();pack.A=ADC;sei();*/
				uint16_t ct=(temperatureC*100.0)+32767;
				cli();pack.A=ct;sei();
				wdcounter=10;
				alarmflag=0;
				
				if (pack.CSA2&0x08)  //AEH
					if (pack.bytes[1]>pack.HA) {alarmflag=1;pack.CSA2|=0x20;}
				if (pack.CSA2&0x04)  //AEL
					if (pack.bytes[1]<pack.LA) {alarmflag=1;pack.CSA2|=0x10;}
			}

			if (pack.convc1&2) {
				/*if (pack.CSB2&0x01)	ADMUX=1; else ADMUX=0x81;
				_delay_us(100);
				ADCSRA|=(1<<ADSC);
				while ((ADCSRA&(1<<ADSC)));
				cli();pack.B=ADC;sei();*/
				wdcounter=10;
				uint16_t hb=humidityRH*100;
				cli();pack.B=hb;sei();
				if (pack.CSB2&0x08)  //AEH
					if (pack.bytes[1]>pack.HB) {alarmflag=1;pack.CSB2|=0x20;}
				if (pack.CSB2&0x04)  //AEL
					if (pack.bytes[1]<pack.LB) {alarmflag=1;pack.CSB2|=0x10;}
			}

			if (pack.convc1&4) {
				/*if (pack.CSC2&0x01)	ADMUX=2; else ADMUX=0x82;
				_delay_us(100);
				ADCSRA|=(1<<ADSC);
				while ((ADCSRA&(1<<ADSC)));
				cli();pack.C=ADC;sei();*/
				wdcounter=10;
				cli();pack.C=l;sei();
				if (pack.CSC2&0x08)  //AEH
					if (pack.bytes[1]>pack.HC) {alarmflag=1;pack.CSC2|=0x20;}
				if (pack.CSC2&0x04)  //AEL
					if (pack.bytes[1]<pack.LC) {alarmflag=1;pack.CSC2|=0x10;}
			} 
			if (pack.convc1&8) {
				/*if (pack.CSD2&0x01)	ADMUX=3; else ADMUX=0x83;
				_delay_us(100);
				ADCSRA|=(1<<ADSC);
				while ((ADCSRA&(1<<ADSC)));
				cli();pack.D=ADC;sei();*/
				wdcounter=10;
				uint16_t hd=P/100.0*32.0;
				cli();pack.D=hd;sei();
				if (pack.CSD2&0x08)  //AEH
					if (pack.bytes[1]>pack.HD) {alarmflag=1;pack.CSD2|=0x20;}
				if (pack.CSD2&0x04)  //AEL
					if (pack.bytes[1]<pack.LD) {alarmflag=1;pack.CSD2|=0x10;}
			}
			
			EXTERN_SLEEP();
			//PORTB&=~(1<<PINB1);
		}

		/*uint8_t bb=1;
		for(uint8_t i=0;i<4;i++) {
			if (pack.bytes[8+i*2]&0x80) {  //Chanel as output
				if (pack.bytes[8+i*2]&0x40) {
					DDRA|=bb;
				} else {
					DDRA&=~bb;
				}
			} else {
				DDRA&=~bb;
			}
			bb=bb*2;
		}*/
		
		OWST_MAIN_END
   }


}