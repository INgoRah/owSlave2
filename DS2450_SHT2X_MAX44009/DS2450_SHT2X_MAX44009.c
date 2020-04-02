
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
#include "../common/I2C/TWI_Master.h"
#include "../common/I2C/SHT2x.h"
#include "../common/I2C/MAX44009.h"
extern void OWINIT();
extern void EXTERN_SLEEP();

uint8_t owid[8]={0x20, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x02, 0x5D};/**/
uint8_t config_info[26]={0x01,14,0x04,0x08, 0x03,15,0,0,2,7,7,15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

#if (owid>128) 
#error "Variable not correct"
#endif

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;
volatile uint8_t wdcounter=10;


#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
#else
ISR(WDT_vect) {
#endif 
	wdcounter++;
	if (reset_indicator==1) reset_indicator++;
	else if (reset_indicator==2) mode=0;
}

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



volatile int16_t am2302_temp;
volatile uint16_t am2302_hum;


uint8_t userRegister[1];
int16_t sRH,sT;
volatile double temperatureC,humidityRH;
volatile double l;
uint32_t P;
int32_t t;
uint8_t max_adr=0;



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
    PORTB=0xFF-(1<<PORTB0); //Schalter kann gegen Masse sein und zieht dann immer Strom
	DDRB|=(1<<PORTB0); //Als Ausgang und 0
	PORTA=0xFF;  //All Pull up;
	 PRR|=(1<<PRADC);  // adc for save Power

	ACSR|=(1<<ACD);  //Disable Comparator
	OWINIT();

	MCUCR &=~(1<<PUD); //All Pins Pullup...
	MCUCR |=(1<<BODS);
	WDTCSR |= ((1<<WDCE) );   // Enable the WD Change Bit//| (1<<WDE)
	WDTCSR |=   (1<<WDIE) |              // Enable WDT Interrupt
	(1<<WDP3) | (1<<WDP0);   // Set Timeout to ~8 seconds
	
	gcontrol=1;
	
	TWI_Master_Initialise();
	SHT2x_SoftReset();
	SHT2x_ReadUserRegister(userRegister);
	//(userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
	SHT2x_WriteUserRegister(userRegister); //write changed user reg
	// --- measure humidity with "Hold Master Mode (HM)" ---
	//SHT2x_MeasurePoll(HUMIDITY, &sRH);
	// --- measure temperature with "Polling Mode" (no hold master) ---
	//SHT2x_MeasurePoll(TEMP, &sT);
	//-- calculate humidity and temperature --
	//temperatureC = SHT2x_CalcTemperatureC(sT);
	//humidityRH = SHT2x_CalcRH(sRH);
	if (checkMAX44009(0)) max_adr=0; else max_adr=1 ;
	
	sei();
	
	//DDRB|=(1<<PINB1);

    while(1)   {
	
	if (wdcounter>3) {	
		SHT2x_MeasurePoll(HUMIDITY, &sRH);
		// --- measure temperature with "Polling Mode" (no hold master) ---
		SHT2x_MeasurePoll(TEMP, &sT);
		//-- calculate humidity and temperature --
		temperatureC = SHT2x_CalcTemperatureC(sT);
		humidityRH = SHT2x_CalcRH(sRH);

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
				cli();pack.B=humidityRH*100;sei();
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
				cli();pack.D=0;sei();
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