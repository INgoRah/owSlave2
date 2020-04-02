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

#define _CPULLUP_

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>



extern void OWINIT();

uint8_t owid[8]={0x1D, 0xA2, 0xD9, 0x84, 0x00, 0x26, 0x02, 0x5C};/**/
uint8_t config_info[26]={9,13,9,13,9,13,9,13,0x02,19,19,19,19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //+2 for CRC

	

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;






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
volatile uint8_t changefromeeprom;



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
	if (((PIN_REG&PIN_CH0)==0)&&((istat&PIN_CH0)==PIN_CH0)) {	counters.c32[0]++;	}
	if (((PIN_REG&PIN_CH1)==0)&&((istat&PIN_CH1)==PIN_CH1)) {	counters.c32[1]++;	}
	istat=PIN_REG;
	changefromeeprom=1;
}


ISR(ANA_COMP_vect) {
	if (changefromeeprom==0) return;
	if ((ACSR&(1<<ACO))!=0) {
		_delay_ms(5);
		if ((ACSR&(1<<ACO))!=0) {
			CLKPR=0x80;//Switch to 4 MHz 
			CLKPR=01;			
			
			PORTB|=(1<<PINB1);
			EEARH=0;
			for(uint8_t i=0;i<16;i++) {
				uint8_t addr=i^0x0C;
				while(EECR & (1<<EEPE));
				EECR = (0<<EEPM1)|(0<<EEPM0);
				EEARL = i;
				EEDR = counters.c8[addr];
				EECR |= (1<<EEMPE);
				EECR |= (1<<EEPE);
			}
			changefromeeprom=0;
			PORTB&=~(1<<PINB1);
			CLKPR=0x80;
			CLKPR=0;
			GIFR|=(1<<INTF0);
		}
	}
	
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
    PRR|=(1<<PRUSI)|(1<<PRADC);  //Switch off usi and adc for save Power
	OWINIT();
	
	pack.zero=0;
	counters.c32[0]=0;
	counters.c32[2]=0;
	counters.c32[1]=0;
	counters.c32[3]=0;
	changefromeeprom=1;
	ACSR|=(1<<ACD);  //Disable Comparator
	ADCSRB|=(1<<ACME); //Disable Analog multiplexer
	MCUCR &=~(1<<PUD); //All Pins Pullup...
	
#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)

	PORTB|=0xFF-(1<<PINB2); //Make PullUp an all Pins but not OW_PIN
	PORTA|=0xFF;
	PORTA&=~(1<<PINA2); //AIN1 
#ifndef _CPULLUP_
	PORTA&=~((1<<PINA4)|(1<<PINA5));
	PORTA&=~((1<<PINA6)|(1<<PINA7));
 #endif

	GIMSK|=(1<<PCIE0);
	PCMSK0=(1<<PCINT4)|(1<<PCINT5)
		|(1<<PCINT6)|(1<<PCINT7);
			
	
	istat=PINB;
#endif

	EEARH=0;
		
	uint8_t addr;
	
	if (testSW()) {  //Jumper gesetzt ->Ruecksetzen
		for(uint8_t i=0;i<16;i++) {
			while(EECR & (1<<EEPE));
			EECR = (0<<EEPM1)|(0<<EEPM0);
			EEARL = i;
			EEDR = 0;
			EECR |= (1<<EEMPE);
			EECR |= (1<<EEPE);
		}	
	}	
	
	
	for(uint8_t i=0;i<16;i++) {
		addr=i^0x0C;
		while(EECR & (1<<EEPE));   
		EEARL=i;
		EECR |= (1<<EERE);
		counters.c8[addr]=EEDR;
	}
	changefromeeprom=0;  //Daten neu eingelesen
	for (uint8_t i=0;i<4;i++) {
		if (counters.c32[i]==0xFFFFFFFF) {
			counters.c32[i]=0;
			changefromeeprom=1;  //Daten geaendert
		}
			//counters.c32[i]=0;
	}
	

		/*for(uint8_t i=0;i<16;i++) {
			while(EECR & (1<<EEPE));
			EECR = (1<<EEPM0);
			EEARL = i;
			EECR |= (1<<EEMPE);
			EECR |= (1<<EEPE);
		}*/
	
   


	DIDR0|=(1<<ADC2D)|(1<<ADC1D); // Disable Digital input on Analog AIN0/AIN1  (PINA1 / PINA2)
	ACSR&=~(1<<ACD);
	ACSR|=(1<<ACIE)|(1<<ACIS1)|(1<<ACIS0)|(1<<ACBG); //Enabble comperator interrupt Rising edge....(1<<ACIS0)

	sei();
	DDRB|=(1<<PINB1);
	PORTB&=~(1<<PINB1);
    while(1)   {
		
		MCUCR|=(1<<SE);
		MCUCR&=~(1<<SM1); 
		asm("SLEEP");
   }
}