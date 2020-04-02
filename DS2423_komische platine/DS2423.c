
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




extern void OWINIT();

uint8_t owid[8]={0x1D, 0xA2, 0xD9, 0x84, 0x00, 0x26, 0x02, 0x5C};/**/
uint8_t config_info[26]={9,13,9,13,9,13,9,13,0x02,19,19,19,19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	

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

typedef union {
	uint32_t c32[4];
	uint8_t c8[16];
} counters_t;

volatile counters_t counters;

volatile uint8_t istat;
volatile uint8_t changefromeeprom;
#define PCINT_VECTOR PCINT0_vect
#define PIN_REG PINA
#define PIN_DDR DDRA
#define PIN_CH0 (1<<PINA3)
#define PIN_CH1 (1<<PINA4)
#define PIN_CH2 (1<<PINA1)
#define PIN_CH3 (1<<PINA2)
#define LPIN_CH2 (1<<PINB0)
#define LDD_CH2 DDRB
#define LPORT_CH2 PORTB
#define LPIN_CH3 (1<<PINA5)
#define LDD_CH3 DDRA
#define LPORT_CH3 PORTA
#define LPIN_CH0 (1<<PINA7)
#define LDD_CH0 DDRA
#define LPORT_CH0 PORTA
#define LPIN_CH1 (1<<PINB1)
#define LDD_CH1 DDRB
#define LPORT_CH1 PORTB
#define PORT_EE PINA //WARNING have to be the same PORT like PINREG because of istat
#define PIN_EE (1<<PINA0)
#define TEST_TIMER  ((TIMSK0 & (1<<TOIE0))==0)



ISR(PCINT0_vect) {
	if (((PIN_REG&PIN_CH0)==0)&&((istat&PIN_CH0)==PIN_CH0)) {	counters.c32[0]++;	LPORT_CH2|=LPIN_CH2;}
	if (((PIN_REG&PIN_CH1)==0)&&((istat&PIN_CH1)==PIN_CH1)) {	counters.c32[1]++;	LPORT_CH2|=LPIN_CH2;}
	if (((PIN_REG&PIN_CH2)==0)&&((istat&PIN_CH2)==PIN_CH2)) {	counters.c32[2]++;	LPORT_CH2|=LPIN_CH2;}
	if (((PIN_REG&PIN_CH3)==0)&&((istat&PIN_CH3)==PIN_CH3)) {	counters.c32[3]++;	LPORT_CH2|=LPIN_CH2;}
	changefromeeprom=1;
/*	if (((PORT_EE&PIN_EE)==0)&&((istat&PIN_EE)==PIN_EE))  {
		
		_delay_ms(50);
	if (((PORT_EE&PIN_EE)==0))  {

		

		LPORT_CH1|=LPIN_CH1;
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
		_delay_ms(250);
		LPORT_CH1&=~LPIN_CH1;

	}
	GIFR|=(1<<PCIF0);
	}*/
	if ((((PINA&(1<<PINA6)))==0)&&((istat&(1<<PINA6))==(1<<PINA6)))   {
		_delay_ms(100);
		if (((PINA&(1<<PINA6)))==0) {
			LPORT_CH3|=LPIN_CH3;
			_delay_ms(100);

			counters.c32[0]=0;
			counters.c32[2]=0;
			counters.c32[1]=0;
			counters.c32[3]=0;
			LPORT_CH3&=~LPIN_CH3;
			}
		GIFR|=(1<<PCIF0);

	}
	istat=PIN_REG;
	
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

	PORTB|=0xFF-(1<<PINB2); //Make PullUp an all Pins but not OW_PIN
	PORTA|=0xFF; 
	PORTA&=~((1<<PINA1)|(1<<PINA2)|(1<<PINA3)|(1<<PINA4));
				
	GIMSK|=(1<<PCIE0);
	PCMSK0=(1<<PCINT1)|(1<<PCINT2)|(1<<PCINT3)|(1<<PCINT4);
	//Spannungsdetektor
	DDRA&=~(1<<PINA0);
	PCMSK0|=(1<<PCINT0);
	PORTA&=~(1<<PINA0);
	//Taster
	DDRA&=~(1<<PINA6);
	PCMSK0|=(1<<PCINT6);
	PORTA|=(1<<PINA6); //Pullup



	LDD_CH0|=LPIN_CH0;
	LPORT_CH0&=~LPIN_CH0;
	LDD_CH1|=LPIN_CH1;
	LPORT_CH1&=~LPIN_CH1;
	LDD_CH2|=LPIN_CH2;
	LPORT_CH2&=~LPIN_CH2;
	LDD_CH3|=LPIN_CH3;
	LPORT_CH3&=~LPIN_CH3;



	LPORT_CH0|=LPIN_CH0;
	_delay_ms(500);
	LPORT_CH0&=~LPIN_CH0;
	
	istat=PINA;
	EEARH=0;
	uint8_t addr;
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

 	DIDR0|=(1<<ADC0D); // Disable Digital input AD0
 	ACSR&=~(1<<ACD); //Enable Comperator
	ADCSRB|=(1<<ACME); // Set other negative input
	ADCSRA&=~(1<<ADEN); //Disable A/D for change the negetive comperator input
	ADMUX&=~((1<<MUX0)|(1<<MUX1)|(1<<MUX2)|(1<<MUX3)|(1<<MUX4)|(1<<MUX5)); //Set do ADC0
 	ACSR|=(1<<ACIE)|(1<<ACIS1)|(1<<ACIS0)|(1<<ACBG); //Enabble comperator interrupt Rising edge....(1<<ACIS0)
  
	sei();
    while(1)   {
	    if (LPORT_CH2&LPIN_CH2) {
			_delay_ms(50);
			LPORT_CH2&=~LPIN_CH2;
		}
		MCUCR|=(1<<SE);
		MCUCR&=~(1<<SM1);
		asm("SLEEP");
		
   }
}