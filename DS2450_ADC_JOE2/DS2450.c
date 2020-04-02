
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
#include "../common/owSlave_tools.h"


OWST_EXTERN_VARS

uint8_t owid[8]={0x20, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x02, 0x5D};/**/
uint8_t config_info[26]={0x06,0x09,0x06,9,0x06,9,0x06,9,0x02,20,20,20,20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

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

OWST_ADC_CONF16_FUNC
OWST_ADC_CONF16_OSS_FUNC
OWST_TESTSW

#define LPIN_CH2 (1<<PINB1)#define LDD_CH2 DDRB#define LPORT_CH2 PORTB#define LED2_ON LPORT_CH2&=~LPIN_CH2;
#define LED2_OFF LPORT_CH2|=LPIN_CH2;

int main(void){
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
	OWST_INIT_ADC_ON
	OWINIT();

	OWST_WDR_CONFIG8
	gcontrol=1;
	OWST_INIT_ADC
	AD_DDR&=~(1<<PINA4);
	AD_PORT&=~(1<<PINA4);
	AD_DDR&=~(1<<PINA5);
	AD_PORT&=~(1<<PINA5);
	AD_DDR&=~(1<<PINA6);
	AD_PORT&=~(1<<PINA6);
	AD_DDR&=~(1<<PINA7);
	AD_PORT&=~(1<<PINA7);
	DIDR0|=((1<<ADC4D)|(1<<ADC5D)|(1<<ADC6D)|(1<<ADC7D));
	sei();
	wdcounter=5;
	//DDRB|=(1<<PINB1);
	LDD_CH2 |=LPIN_CH2;
	volatile double VCC;
	volatile double VAD_A,VAD_B,VAD_C,VAD_D;
    while(1)   {
		if (wdcounter>3) {
			LED2_ON
			wdcounter=0;
			ADMUX=OWST_ADCIN_REFINT;
			_delay_us(100);
			VCC=owst_ADC_runf();
			VCC=(1.079*65472.0)/VCC;
			//if (VCC>5.1) VCC=5.1;
			//VAD=VCC*65535.0/5.1;
			//if (VAD>65535) VAD=65535;
			if (testSW()) {	 //Zwei differential Eingaenge
				VAD_C=0;
				ADMUX=0b0011010; //ADC4 + ADC5 - Gain 1
				_delay_us(100);
				VAD_A=owst_ADC_runf();
				if (VAD_A<3100) {
					ADMUX=0b0011011; //ADC0 + ADC1 - Gain 20
					_delay_us(100);
					VAD_A=owst_ADC_runf();
					VAD_A=VCC/20.0*VAD_A/65472.0;
					//VAD_A=VCC/20.0;
					VAD_C+=12850;
				} else {
					VAD_A=owst_ADC_OSS_runf();
					VAD_A=VCC*VAD_A/65472.0;
					VAD_C+=0;
				}
				if (pack.CSA2&0x01) VAD_A=VAD_A*65535.0/5.1; else VAD_A=VAD_A*65535.0/2.55;
				if (VAD_A>65535) VAD_A=65535;
				///_------------------------ B---------------------
				ADMUX=0b011110; //ADC6 + ADC7 - Gain 1
				_delay_us(100);
				VAD_B=owst_ADC_runf();
				if (VAD_B<3100) {
					ADMUX=0b011111; //ADC6 + ADC7 - Gain 20
					_delay_us(100);
					VAD_B=owst_ADC_runf();
					VAD_B=VCC/20.0*VAD_B/65472.0;
					//VAD_B=VAD_B20.0;
					VAD_C+=12850*2;
				} else {
					VAD_B=owst_ADC_OSS_runf();
					VAD_B=VCC*VAD_B/65472.0;
					VAD_C+=0;
				}
				if (pack.CSB2&0x01) VAD_B=VAD_B*65535.0/5.1; else VAD_B=VAD_B*65535.0/2.55;
				//VAD_B=VAD_B*65535.0/5.1;
				if (VAD_B>65535) VAD_B=65535;
				VAD_D=VCC*65535.0/5.1;
				if (VAD_D>65535) VAD_D=65535;

			} else { // 4 Eingaenge gegen Masse
				ADMUX=OWST_ADCIN_PA4;
				_delay_us(100);
				VAD_A=owst_ADC_OSS_runf();
				VAD_A=VCC*VAD_A/65472.0;
				if (pack.CSA2&0x01) VAD_A=VAD_A*65535.0/5.1; else VAD_A=VAD_A*65535.0/2.55;
				if (VAD_A>65535) VAD_A=65535;

				ADMUX=OWST_ADCIN_PA5;
				_delay_us(100);
				VAD_B=owst_ADC_OSS_runf();
				VAD_B=VCC*VAD_B/65472.0;
				if (pack.CSB2&0x01) VAD_B=VAD_B*65535.0/5.1; else VAD_B=VAD_B*65535.0/2.55;
				if (VAD_B>65535) VAD_B=65535;

				ADMUX=OWST_ADCIN_PA6;
				_delay_us(100);
				VAD_C=owst_ADC_OSS_runf();
				VAD_C=VCC*VAD_C/65472.0;
				if (pack.CSC2&0x01) VAD_C=VAD_C*65535.0/5.1; else VAD_C=VAD_C*65535.0/2.55;
				if (VAD_C>65535) VAD_C=65535;

				ADMUX=OWST_ADCIN_PA7;
				_delay_us(100);
				VAD_D=owst_ADC_OSS_runf();
				VAD_D=VCC*VAD_D/65472.0;
				if (pack.CSD2&0x01) VAD_D=VAD_D*65535.0/5.1; else VAD_D=VAD_D*65535.0/2.55;
				if (VAD_D>65535) VAD_D=65535;
			}
			LED2_OFF


		}

		if (gcontrol) {
			//PORTB|=(1<<PINB1);
			uint8_t bb=1;
			uint8_t bb1=1;
			for(uint8_t i=0;i<4;i++){ //Initialisieren mit 0 oder FF oder letzter Wert
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
				//if (pack.CSA2&0x01)	ADMUX=0; else ADMUX=0x80;
				cli();pack.A=VAD_A;sei();
				alarmflag=0;
				if (pack.CSA2&0x08)  //AEH
					if (pack.bytes[1]>pack.HA) {alarmflag=1;pack.CSA2|=0x20;}
				if (pack.CSA2&0x04)  //AEL
					if (pack.bytes[1]<pack.LA) {alarmflag=1;pack.CSA2|=0x10;}
			}

			if (pack.convc1&2) {
				//if (pack.CSB2&0x01)	ADMUX=1; else ADMUX=0x81;
				cli();pack.B=VAD_B;sei();
				if (pack.CSB2&0x08)  //AEH
					if (pack.bytes[1]>pack.HB) {alarmflag=1;pack.CSB2|=0x20;}
				if (pack.CSB2&0x04)  //AEL
					if (pack.bytes[1]<pack.LB) {alarmflag=1;pack.CSB2|=0x10;}
			}

			if (pack.convc1&4) {
				//if (pack.CSC2&0x01)	ADMUX=2; else ADMUX=0x82;
				cli();pack.C=VAD_C;sei();
				if (pack.CSC2&0x08)  //AEH
					if (pack.bytes[1]>pack.HC) {alarmflag=1;pack.CSC2|=0x20;}
				if (pack.CSC2&0x04)  //AEL
					if (pack.bytes[1]<pack.LC) {alarmflag=1;pack.CSC2|=0x10;}
			} 
			if (pack.convc1&8) {
				cli();pack.D=VAD_D;sei();
				if (pack.CSD2&0x08)  //AEH
					if (pack.bytes[1]>pack.HD) {alarmflag=1;pack.CSD2|=0x20;}
				if (pack.CSD2&0x04)  //AEL
					if (pack.bytes[1]<pack.LD) {alarmflag=1;pack.CSD2|=0x10;}
			}
			
			EXTERN_SLEEP();
			//PORTB&=~(1<<PINB1);
		}

		uint8_t bb=1;
		for(volatile uint8_t i=0;i<4;i++) {
			if (pack.bytes[8+i*2]&0x80) {  //Chanel as output
				if (pack.bytes[8+i*2]&0x40) {
				//	AD_DDR|=bb;
				} else {
					cli();
				//	AD_DDR&=~bb;
					sei();
				}
			} else {
				cli();
			//	AD_DDR&=~bb;
				sei();
			}
			bb=bb*2;
		}
		OWST_MAIN_END 
   }


}