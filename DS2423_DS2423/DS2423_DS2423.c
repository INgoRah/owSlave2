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
//    software must display the following acknowledgment: This product
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

//!!!!!Max Program size 7551 Byte
#define _CPULLUP_
#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

extern void OWINIT(void);
extern void EXTERN_SLEEP(void);


//#define FHEM_PLATINE
//#define JOE_M
#define W1DAQ

volatile uint8_t owid1[8]={0x1D, 0x40, 0xDA, 0x84, 0x00, 0x00, 0x05, 0xBD};/**/
volatile uint8_t owid2[8]={0x1D, 0x41, 0xDA, 0x84, 0x00, 0x00, 0x05, 0x8A};/**/
#if RAMEND>260 //defined(__AVR_ATtiny84__)   ||defined(__AVR_ATtiny84A__)
uint8_t config_info1[26]={9,13,9,13,9,13,9,13,0x02,19,19,19,19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //+2 for CRC
uint8_t config_info2[26]={9,13,9,13,9,13,9,13,0x02,19,19,19,19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //+2 for CRC
#endif
#if (owid>128)
#error "Variable not correct"
#endif

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
counterpack_t pack1;

//The Memory of both  Chips is the same, beause of the small memory of ATTiny44
#define pack2 pack1

volatile uint8_t lastcps;
typedef union {
	uint32_t c32[4];
	uint8_t c8[16];
} counters_t;

volatile counters_t counters1,counters2;


volatile uint8_t istat;
volatile uint8_t changefromeeprom;

#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) ||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define PCINT_VECTOR PCINT0_vect
#define PIN_REG PINA
#define PIN_DDR DDRA

#ifdef FHEM_PLATINE
#define PIN_CH3 (1<<PINA2)
#define PIN_CH2 (1<<PINA1)
#define PIN_CH1 (1<<PINA4)
#define PIN_CH0 (1<<PINA3)
//LEDS#define LPIN_CH2 (1<<PINB0)#define LDD_CH2 DDRB#define LPORT_CH2 PORTB#define LPIN_CH3 (1<<PINA5)#define LDD_CH3 DDRA#define LPORT_CH3 PORTA#define LPIN_CH0 (1<<PINA7)#define LDD_CH0 DDRA#define LPORT_CH0 PORTA#define LPIN_CH1 (1<<PINB1)#define LDD_CH1 DDRB#define LPORT_CH1 PORTB

#define LED2_ON LPORT_CH2|=LPIN_CH2;
#endif

#ifdef JOE_M
#define LED2_ON
#define PIN_CH2 (1<<PINA4)
#define PIN_CH3 (1<<PINA5)
#define PIN_CH0 (1<<PINA6)
#define PIN_CH1 (1<<PINA7)
#endif

#ifdef W1DAQ
#define PIN_CH3 (1<<PINA1)
#define PIN_CH2 (1<<PINA0)
#define PIN_CH1 (1<<PINA7)
#define PIN_CH0 (1<<PINA3)
//LEDS#define LPIN_CH0 (1<<PINB1)#define LDD_CH0 DDRB#define LPORT_CH0 PORTB#define LPIN_CH1 (1<<PINB1)#define LDD_CH1 DDRB#define LPORT_CH1 PORTB#define LPIN_CH2 (1<<PINB1)#define LDD_CH2 DDRB#define LPORT_CH2 PORTB#define LPIN_CH3 (1<<PINB1)#define LDD_CH3 DDRB#define LPORT_CH3 PORTB
#define LED2_ON LPORT_CH2&=~LPIN_CH2;
#endif


#define TEST_TIMER  ((TIMSK0 & (1<<TOIE0))==0)

#endif


ISR(PCINT0_vect) {
	if (((PIN_REG&PIN_CH2)==0)&&((istat&PIN_CH2)==PIN_CH2)) {	counters1.c32[2]++;LED2_ON}
	if (((PIN_REG&PIN_CH3)==0)&&((istat&PIN_CH3)==PIN_CH3)) {	counters1.c32[3]++;	LED2_ON}
	if (((PIN_REG&PIN_CH0)==0)&&((istat&PIN_CH0)==PIN_CH0)) {	counters2.c32[2]++;	LED2_ON}
	if (((PIN_REG&PIN_CH1)==0)&&((istat&PIN_CH1)==PIN_CH1)) {	counters2.c32[3]++;	LED2_ON}
	
	//Reset Switch on the FHEM_BOARD
	#ifdef FHEM_PLATINE  //clear counter
	if ((((PINA&(1<<PINA6)))==0)&&((istat&(1<<PINA6))==(1<<PINA6)))   {		_delay_ms(100);		if (((PINA&(1<<PINA6)))==0) {			LPORT_CH3|=LPIN_CH3;			_delay_ms(100);			counters1.c32[2]=0;			counters1.c32[3]=0;			counters2.c32[2]=0;			counters2.c32[3]=0;			//counters1.c32[0]=0;			//counters1.c32[1]=0;			//counters2.c32[0]=0;			//counters2.c32[1]=0;			//count Resets			counters1.c32[0]++;			counters2.c32[0]++;			LPORT_CH3&=~LPIN_CH3;		}		GIFR|=(1<<PCIF0);	}	#endif	istat=PIN_REG;
	changefromeeprom=1;

}


ISR(ANA_COMP_vect) {
	if (changefromeeprom==0) return;
	if ((ACSR&(1<<ACO))!=0) {
		_delay_ms(5);
		if ((ACSR&(1<<ACO))!=0) {
			//Count Savings
			counters1.c32[1]++;			counters2.c32[1]=counters1.c32[1];			CLKPR=0x80;//Switch to 4 MHz
			CLKPR=01;
			
			PORTB|=(1<<PINB1);
			EEARH=0;
			for(uint8_t i=0;i<16;i++) {
				uint8_t addr=i;
				while(EECR & (1<<EEPE));
				EECR = (0<<EEPM1)|(0<<EEPM0);
				EEARL = i;
				EEDR = counters1.c8[addr];
				EECR |= (1<<EEMPE);
				EECR |= (1<<EEPE);
			}
			for(uint8_t i=16;i<32;i++) {
				uint8_t addr=i-16;
				while(EECR & (1<<EEPE));
				EECR = (0<<EEPM1)|(0<<EEPM0);
				EEARL = i;
				EEDR = counters2.c8[addr];
				EECR |= (1<<EEMPE);
				EECR |= (1<<EEPE);
			}
			changefromeeprom=0;
			PORTB&=~(1<<PINB1);
			CLKPR=0x80;
			CLKPR=0;
#ifdef FHEM_PLATINE
			LPORT_CH1|=LPIN_CH1;
#endif
#ifdef W1DAQ
			LPORT_CH1&=~LPIN_CH1;
#endif
			GIFR|=(1<<INTF0);
		}
	}
	
}


int main(void){
	#ifdef FHEM_PLATINE
	PRR|=(1<<PRUSI);  //Switch off usi, dont switch of ADC cause Multiplexer is used for the correct AIN1 pin
	#else
	PRR|=(1<<PRUSI)|(1<<PRADC);  //Switch off usi and adc for save Power
	#endif
	OWINIT();
	
	pack1.zero=0;
	pack2.zero=0;
	counters1.c32[0]=0;
	counters1.c32[2]=0;
	counters1.c32[1]=0;
	counters1.c32[3]=0;
	counters2.c32[0]=0;
	counters2.c32[2]=0;
	counters2.c32[1]=0;
	counters2.c32[3]=0;
	changefromeeprom=1;
	
	#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)

	PORTB|=0xFF-(1<<PINB2); //Make PullUp an all Pins but not OW_PIN
	PORTA|=0xFF;

	#ifndef _CPULLUP_  // pullup
	PORTA&=~(PIN_CH0|PIN_CH1);
	PORTA&=~(PIN_CH2|PIN_CH3);
	#endif

	#if defined(FHEM_PLATINE) || defined(W1DAQ)  //LEDs
	LDD_CH0|=LPIN_CH0;	LPORT_CH0&=~LPIN_CH0;	LDD_CH1|=LPIN_CH1;	LPORT_CH1&=~LPIN_CH1;	LDD_CH2|=LPIN_CH2;	LPORT_CH2&=~LPIN_CH2;	LDD_CH3|=LPIN_CH3;	LPORT_CH3&=~LPIN_CH3;
	#endif

	GIMSK|=(1<<PCIE0);
	PCMSK0=(PIN_CH0|PIN_CH1|PIN_CH2|PIN_CH3); //Nicht ganz korrekt aber die Bits liegen gleich

	
	
	istat=PINA;
	#endif

	EEARH=0;
	uint8_t addr;
	for(uint8_t i=0;i<16;i++) {
		addr=i;
		while(EECR & (1<<EEPE));
		EEARL=i;
		EECR |= (1<<EERE);
		counters1.c8[addr]=EEDR;
	}
	for(uint8_t i=16;i<32;i++) {
		addr=i-16;
		while(EECR & (1<<EEPE));
		EEARL=i;
		EECR |= (1<<EERE);
		counters2.c8[addr]=EEDR;
	}
	changefromeeprom=0;  //Daten neu eingelesen
	for (uint8_t i=0;i<4;i++) {
		if (counters1.c32[i]==0xFFFFFFFF) {
			counters1.c32[i]=0;
			changefromeeprom=1;  //Daten geaendert
		}
		//counters.c32[i]=0;
	}
	for (uint8_t i=0;i<4;i++) {
		if (counters2.c32[i]==0xFFFFFFFF) {
			counters2.c32[i]=0;
			changefromeeprom=1;  //Daten geaendert
		}
		//counters.c32[i]=0;
	}

	//ACSR|=(1<<ACD);  //Disable Comparator
	ADCSRB|=(1<<ACME); //Disable Analog multiplexer
	MCUCR &=~(1<<PUD); //All Pins Pullup...
#ifdef FHEM_PLATINE
	DIDR0|=(1<<ADC0D);
	PORTA&=~(1<<PINA0);//Disable Pullup
#else
	DIDR0|=(1<<ADC2D)|(1<<ADC1D); // Disable Digital input on Analog AIN0/AIN1  (PINA1 / PINA2)
	PORTA&=~(1<<PINA2); //AIN1
#endif
	ACSR&=~(1<<ACD);
	ACSR|=(1<<ACIE)|(1<<ACIS1)|(1<<ACIS0)|(1<<ACBG); //Enabble comperator interrupt Rising edge....(1<<ACIS0) -> minus of Comperator falls down -> output of Comperator rises
#ifdef FHEM_PLATINE
	//Switch std AIN1 to A0
	ADCSRA&=~(1<<ADEN);
	ADCSRB=(1<<ACME);
	ADMUX=0;
	//Taster
	DDRA&=~(1<<PINA6);	PCMSK0|=(1<<PCINT6);	PORTA|=(1<<PINA6); //Pullup


	LPORT_CH0|=LPIN_CH0;	_delay_ms(500);	LPORT_CH0&=~LPIN_CH0;
#endif
#ifdef W1DAQ
	LPORT_CH0&=~LPIN_CH0;
	_delay_ms(500);	LPORT_CH0|=LPIN_CH0;#endif
	sei();
	while(1)   {
#ifdef FHEM_PLATINE
		if (LPORT_CH2&LPIN_CH2) {			_delay_ms(50);			LPORT_CH2&=~LPIN_CH2;		}
		if (LPORT_CH1&LPIN_CH1) {			_delay_ms(50);			LPORT_CH1&=~LPIN_CH1;		}
#endif
#ifdef W1DAQ
		if ((LPORT_CH2&LPIN_CH2)==0) {			_delay_ms(50);			LPORT_CH2|=LPIN_CH2;		}
#endif
#ifndef FHEM_PLATINE
		if ((PINB&(1<<PORTB0))==0) {  //Jumper gesetzt ->Ruecksetzen
			if ((counters1.c32[2]!=0)||(counters1.c32[3]!=0)||(counters2.c32[2]!=0)||(counters2.c32[3]!=0)) {
				counters1.c32[0]++;				counters2.c32[0]++;				for (uint8_t i=2;i<4;i++) {
					counters1.c32[i]=0;
					counters2.c32[i]=0;
				}
				//count Resets				changefromeeprom=1;
			}
		}
		#endif
		if (((TIMSK0 & (1<<TOIE0))==0)&& (mode==0))	  {
			MCUCR|=(1<<SE)|(1<<SM1);
			
			MCUCR&=~(1<<ISC01);
			} else {
			MCUCR|=(1<<SE);
			MCUCR&=~(1<<SM1);
		}
		asm("SLEEP");

/*
		MCUCR|=(1<<SE);
		MCUCR&=~(1<<SM1);
		asm("SLEEP");*/
	}


}
