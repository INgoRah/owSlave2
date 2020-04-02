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
#include "../common/I2C/HDC1080.h"
#include "../common/calibr.h"

extern void OWINIT();
extern void EXTERN_SLEEP();

uint8_t owid[8]={0x26, 0xA2, 0xD9, 0x84, 0x00, 0x00, 0x05, 0x16};/**/
uint8_t config_info[26]={0x01,0x06, 0x05,0x08, 0x04,0x07, 0x00,0x00, 0x02,11,0x00,11,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;

volatile uint8_t wdcounter=5;


typedef union {

	volatile uint8_t bytes[64];
	struct {
		uint8_t status;  //1
		int16_t temp;  //2
		uint16_t voltage;  //4
		uint16_t current;  //6
		uint8_t threshold; //8
		
		uint8_t page1[8]; //9
		uint8_t page2[8]; //17
		uint8_t page3[8]; //25
		uint8_t page4[8];  //33
		uint8_t page5[8];  //41
		uint8_t page6[8];  //49
		uint8_t page7[8];  //57
		//uint8_t crc;  //65
	};
} pack_t;
volatile pack_t pack;



volatile int16_t am2302_temp;
volatile uint16_t am2302_hum;

uint8_t userRegister[1];
int16_t sRH,sT;
double temperatureC,humidityRH;
volatile double l;

#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
#else
ISR(WDT_vect) {
#endif 
	wdcounter++;
	if (reset_indicator==1) reset_indicator++;
	else if (reset_indicator==2) mode=0;


}

int testSW(void) {
	uint8_t r;
	DDRB&=~(1<<PORTB0);  //Eingang
	 __asm__ __volatile__ ("nop"); 
	 PORTB|=(1<<PORTB0); //Pullup
	 __asm__ __volatile__ ("nop"); 
	 __asm__ __volatile__ ("nop"); 
	 __asm__ __volatile__ ("nop"); 
	 __asm__ __volatile__ ("nop"); 
	 __asm__ __volatile__ ("nop"); 
	 r=PINB&(1<<PORTB0);
	 __asm__ __volatile__ ("nop"); 
	 PORTB&=~(1<<PORTB0); 
	 __asm__ __volatile__ ("nop"); 
	DDRB|=(1<<PORTB0);  //Eingang
	return (r==0);
	 
	 
}

int main(void){
     PRR|=(1<<PRADC);  // adc for save Power
	PORTA=0xFF;
	PORTB=0xFF-(1<<PORTB0); //Schalter kann gegen Masse sein und zieht dann immer Strom
	DDRB|=(1<<PORTB0); //Als Ausgang und 0
	
	
	OWINIT();

	ACSR|=(1<<ACD);  //Disable Comparator
	ADCSRB|=(1<<ACME); //Disable Analog multiplexer
	MCUCR &=~(1<<PUD); //All Pins Pullup...
	MCUCR |=(1<<BODS);

	
	WDTCSR |= ((1<<WDCE) );   // Enable the WD Change Bit//| (1<<WDE)
	WDTCSR |=   (1<<WDIE) |              // Enable WDT Interrupt
	(1<<WDP3) | (1<<WDP0);   // Set Timeout to ~8 seconds
	 if (testSW()) {
		config_info[5]=8;
	}else{
		config_info[5]=7;
	 }
	
    uint8_t i;
    for(i=0;i<64;i++) pack.bytes[i]=0;
	MCUSR=0;
	USI_TWI_Master_Initialise();
	HDC2010_Init();
	HDC2010_Readf(&temperatureC,&humidityRH);
	
	sei();
	
	 while(1)   {
		 if (gcontrol) {
			 wdcounter=3;
			 gcontrol=0;
		 }
		 if (wdcounter>2) {
			 //PORTB|=(1<<PINB1); //Dauer 440ms
			 HDC2010_Readf(&temperatureC,&humidityRH);

			 humidityRH = calibr_hum(temperatureC,-0.2,humidityRH)*10.0;
			 temperatureC-=0.2;
			 temperatureC*=10.0;
			 if (testSW()) {
				 am2302_hum= humidityRH;
				 am2302_temp=temperatureC*25.6;
				 //am2302_temp=am2302_temp-45;
				 config_info[5]=12;	
				 
			 }else{
				 
				double hhum=(1.0546-0.000216*temperatureC)*(humidityRH);
				//am2302_hum=0.318*hhum +76.0;
				am2302_hum=0.31*hhum +80;
				am2302_temp=temperatureC*25.6;
				//am2302_temp=am2302_temp-45;
				config_info[5]=7;
			 }
			 //PORTB&=~(1<<PINB1);
			 wdcounter=0;
		 }

		if (((TIMSK0 & (1<<TOIE0))==0)&& (mode==0))	  {
			MCUCR|=(1<<SE)|(1<<SM1);
			MCUCR&=~(1<<ISC01);
		} else {
			MCUCR|=(1<<SE);
			MCUCR&=~(1<<SM1);
		}
		asm("SLEEP");
	}


}