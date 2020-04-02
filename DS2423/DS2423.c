
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
#define _CPULLUP_

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>



extern void OWINIT();

uint8_t owid[8]={0x1D, 0xA2, 0xD9, 0x84, 0x00, 0x26, 0x02, 0x5C};/**/
uint8_t config_info[26]={9,13,9,13,9,13,9,13,0x02,19,19,19,19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //+2 for CRC

	

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;


volatile uint8_t wdcounter;




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

#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define PCINT_VECTOR PCINT0_vect
#define PIN_REG PINA
#define PIN_DDR DDRA
#define PIN_CH2 (1<<PINA4)
#define PIN_CH3 (1<<PINA5)
#define PIN_CH0 (1<<PINA6)
#define PIN_CH1 (1<<PINA7)
#define PORT_EE PINA //WARNING have to be the same PORT like PINREG because of istat
#define PIN_EE (1<<PINA0)
#define TEST_TIMER  ((TIMSK0 & (1<<TOIE0))==0)

#endif


ISR(PCINT0_vect) {
	if (((PIN_REG&PIN_CH2)==0)&&((istat&PIN_CH2)==PIN_CH2)) {	counters.c32[2]++;	}
	if (((PIN_REG&PIN_CH3)==0)&&((istat&PIN_CH3)==PIN_CH3)) {	counters.c32[3]++;	}
	#ifdef _4_COUNTERS_
	if (((PIN_REG&PIN_CH0)==0)&&((istat&PIN_CH0)==PIN_CH0)) {	counters.c32[0]++;	}
	if (((PIN_REG&PIN_CH1)==0)&&((istat&PIN_CH1)==PIN_CH1)) {	counters.c32[1]++;	}
	#endif
	istat=PIN_REG;
}


int main(void){
    PRR|=(1<<PRUSI)|(1<<PRADC);  //Switch off usi and adc for save Power
	OWINIT();
	
	pack.zero=0;
	counters.c32[0]=0;
	counters.c32[2]=0;
	counters.c32[1]=0;
	counters.c32[3]=0;
	ACSR|=(1<<ACD);  //Disable Comparator
	ADCSRB|=(1<<ACME); //Disable Analog multiplexer
	MCUCR &=~(1<<PUD); //All Pins Pullup...
	

#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)

	PORTB|=0xFF-(1<<PINB2); //Make PullUp an all Pins but not OW_PIN
	PORTA|=0xFF; 
	DDRB |= _BV(PB0) | _BV(PB1);
	PORTB &= ~(_BV(PB0) | _BV(PB1));

	if (PINB&(1<<PINB0)) { 
	} else {//SW set, PINB0 =0 no pullup, clear PORT of ios
		PORTA&=~((1<<PINA4)|(1<<PINA5));
#ifdef _4_COUNTERS_
		PORTA&=~((1<<PINA6)|(1<<PINA7));
#endif
		PORTB&=~(1<<PINB0);  //Disable Pullup io switch to save Power
				
	} 

	GIMSK|=(1<<PCIE0);
	PCMSK0=(1<<PCINT4)|(1<<PCINT5)
#ifdef _4_COUNTERS_
		|(1<<PCINT6)|(1<<PCINT7)
#endif
	;
	istat=PINA;
#endif

	sei();
    while(1)   {
		
		//Test if timer active and no sleep then Idle else Power Down
		if (TEST_TIMER&&(mode==0)) {
			MCUCR|=(1<<SE)|(1<<SM1); //Power Down, only low level on 1-Wire and pin change on PCINT wakes up
			MCUCR&=~(1<<ISC01);
		} else {
			MCUCR|=(1<<SE);
			MCUCR&=~(1<<SM1); 
		}
		asm("SLEEP");
   }
}