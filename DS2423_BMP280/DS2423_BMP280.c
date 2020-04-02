
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

#define _4_COUNTERS_
#define _EEPROM_SAVE_
//#define _CPULLUP_

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "../common/I2C/USI_TWI_Master.h"
#include "../common/I2C/BMP280.h"



extern void OWINIT();
extern void EXTERN_SLEEP();

uint8_t owid[8]={0x1D, 0xA2, 0xD9, 0x84, 0x00, 0x26, 0x02, 0x5C};/**/
uint8_t config_info[26]={0x00,0x00,0x00,0x00,0x01,0x08,0x02,0x08,0x02,0x00,0x00,14,14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	

extern volatile uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;


volatile uint8_t wdcounter;

#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
#else
ISR(WDT_vect) {
#endif 
	wdcounter++;
	//if (reset_indicator==1) reset_indicator++;
	//else if (reset_indicator==2) mode=0;
}


typedef union {
	volatile uint8_t bytes[45];
	struct {
		uint16_t addr;
		uint8_t status;
		uint8_t scratch[32];//3
		uint32_t counter;  //35
		uint32_t zero;   //39
		uint16_t crc;  //43
	};
} counterpack_t;
counterpack_t pack;

volatile uint8_t lastcps;
typedef union {
	uint32_t c32[4];
	uint8_t c8[16];
} counters_t;

volatile counters_t counters;

volatile uint8_t istat;


volatile uint8_t wdcounter;

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
	 
	 
	pack.zero=0;
	counters.c32[0]=0;
	counters.c32[2]=0;
	counters.c32[1]=0;
	counters.c32[3]=0;
	ACSR|=(1<<ACD);  //Disable Comparator
	ADCSRB|=(1<<ACME); //Disable Analog multiplexer
	MCUCR &=~(1<<PUD); //All Pins Pullup...
	
	USI_TWI_Master_Initialise();
	bmp280Init();	

	uint32_t P;
	int32_t t;
	bmp280ConvertInt(&t,&P,1);
	_delay_ms(200);
	bmp280ConvertInt(&t,&P,1);
	counters.c32[3]=P;
	counters.c32[2]=t;
	wdcounter=100;
	sei();
    while(1)   {
		
		if (wdcounter>2) {
			bmp280ConvertInt(&t,&P,1);
			while (mode!=0) ;
			cli();
			counters.c32[3]=P;
			counters.c32[2]=t;
			sei();
			
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