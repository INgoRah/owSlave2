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

//!!!!!Max Program size 7551 Byte
#define _CPULLUP_
#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>  
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include "../common/owSlave_tools.h"


//#define FHEM_PLATINE
#define W1DAQ
//#define JOE_M
volatile uint8_t owid1[8]={0x3A, 0x01, 0xDA, 0x84, 0x00, 0x00, 0x05, 0xA3};/**/
volatile uint8_t owid2[8]={0x3A, 0x02, 0xDA, 0x84, 0x00, 0x00, 0x05, 0xFA};/**/
uint8_t config_info1[26]={0,0,0,0,0,0,0,0,0x02,0,0,0,0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //+2 for CRC
uint8_t config_info2[26]={0,0,0,0,0,0,0,0,0x02,0,0,0,0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //+2 for CRC
	
#if (owid>128) 
#error "Variable not correct"
#endif

OWST_EXTERN_VARS

uint8_t pin_state1;
uint8_t pin_set1;
uint8_t pin_state2;
uint8_t pin_set2;

OWST_WDT_ISR


#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define PCINT_VECTOR PCINT0_vect
#define PIN_REG PINA
#define PIN_DDR DDRA

#ifdef FHEM_PLATINE
#define PIN_PIOA1 (1<<PINA2)
#define PIN_PIOB1 (1<<PINA1)
#define PIN_PIOA2 (1<<PINA3)
#define PIN_PIOB2 (1<<PINA4)
//LEDS#define LPIN_CH2 (1<<PINB0)#define LDD_CH2 DDRB#define LPORT_CH2 PORTB#define LPIN_CH3 (1<<PINA5)#define LDD_CH3 DDRA#define LPORT_CH3 PORTA#define LPIN_CH0 (1<<PINA7)#define LDD_CH0 DDRA#define LPORT_CH0 PORTA#define LPIN_CH1 (1<<PINB1)#define LDD_CH1 DDRB#define LPORT_CH1 PORTB

#define LED2_ON LPORT_CH2|=LPIN_CH2;
#endif

#ifdef JOE_M
#define LED2_ON
#define PIN_PIOA1 (1<<PINA4)
#define PIN_PIOB1 (1<<PINA5)
#define PIN_PIOA2 (1<<PINA6)
#define PIN_PIOB2 (1<<PINA7)
#endif

#ifdef W1DAQ
#define PIN_PIOB1 (1<<PINA1)
#define PIN_PIOA1 (1<<PINA0)
#define PIN_PIOB2 (1<<PINA7)
#define PIN_PIOA2 (1<<PINA3)
//LEDS#define LPIN_CH0 (1<<PINB1)#define LDD_CH0 DDRB#define LPORT_CH0 PORTB#define LPIN_CH1 (1<<PINB1)#define LDD_CH1 DDRB#define LPORT_CH1 PORTB#define LPIN_CH2 (1<<PINB1)#define LDD_CH2 DDRB#define LPORT_CH2 PORTB#define LPIN_CH3 (1<<PINB1)#define LDD_CH3 DDRB#define LPORT_CH3 PORTB
#define LED2_ON LPORT_CH2&=~LPIN_CH2;
#endif



#endif


ISR(PCINT0_vect) {
//	if (((PIN_REG&PIN_CH2)==0)&&((istat&PIN_CH2)==PIN_CH2)) {	counters1.c32[2]++;	LED2_ON}
	//if (((PIN_REG&PIN_CH3)==0)&&((istat&PIN_CH3)==PIN_CH3)) {	counters1.c32[3]++;LED2_ON	}
	if ((PIN_REG&PIN_PIOA1)==0) 	{pin_state1&=~0x1;LED2_ON} else { pin_state1|=0x01;}
	if ((PIN_REG&PIN_PIOB1)==0) 	{pin_state1&=~0x4;LED2_ON} else {pin_state1|=0x04;}
	if ((PIN_REG&PIN_PIOA2)==0) 	{pin_state2&=~0x1;LED2_ON} else {pin_state2|=0x01;}
	if ((PIN_REG&PIN_PIOB2)==0) 	{pin_state2&=~0x4;LED2_ON} else {pin_state2|=0x04;}
	//Reset Switch on the FHEM_BOARD
		GIFR|=(1<<PCIF0);
}





int main(void){
	OWST_INIT_ALL_OFF
	OWINIT();
	

#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)

    OWST_WDR_CONFIG4
	
#ifndef _CPULLUP_  // pullup
	PORTA&=~(PIN_PIOA1|PIN_PIOB1);
	PORTA&=~(PIN_PIOA2|PIN_PIOB2);
#endif

#ifdef FHEM_PLATINE  //LEDs
	LDD_CH0|=LPIN_CH0;	LPORT_CH0&=~LPIN_CH0;	LDD_CH1|=LPIN_CH1;	LPORT_CH1&=~LPIN_CH1;	LDD_CH2|=LPIN_CH2;	LPORT_CH2&=~LPIN_CH2;	LDD_CH3|=LPIN_CH3;	LPORT_CH3&=~LPIN_CH3;
#endif
#ifdef W1DAQ
	LDD_CH2|=LPIN_CH2;	LPORT_CH2&=~LPIN_CH2;#endif


	GIMSK|=(1<<PCIE0);
	PCMSK0=(PIN_PIOA1|PIN_PIOB1|PIN_PIOA2|PIN_PIOB2); //Nicht ganz korrekt aber die Bits liegen gleich

	
	
	#endif
	
	OWST_EN_PULLUP
	
	
#ifdef FHEM_PLATINE

	LPORT_CH0|=LPIN_CH0;	_delay_ms(500);	LPORT_CH0&=~LPIN_CH0;
#endif
	pin_set1=0x0F;
	pin_set2=0x0F;
	//pin_state1=0x00;
	//pin_state2=0x00;
	sei();
	while(1)   {

		if (pin_set1&1) {
			DDRA&=~(PIN_PIOA1); //Eingang
			PORTA|=(PIN_PIOA1); //Pullup
			pin_state1|=2;
		} else {
			DDRA|=(PIN_PIOA1); //Ausgang
			PORTA&=~(PIN_PIOA1); //Gegen masse
			pin_state1&=~2;
		}
		if (pin_set1&2) {
			pin_state1|=8;
			DDRA&=~(PIN_PIOB1); //Eingang
			PORTA|=(PIN_PIOB1); //Pullup
			} else {
			DDRA|=(PIN_PIOB1); //Ausgang
			PORTA&=~(PIN_PIOB1); //Gegen masse
			pin_state1&=~8;
		}
		if (pin_set2&1) {
			pin_state2|=2;
			DDRA&=~(PIN_PIOA2); //Eingang
			PORTA|=(PIN_PIOA2); //Pullup
			} else {
			DDRA|=(PIN_PIOA2); //Ausgang
			PORTA&=~(PIN_PIOA2); //Gegen masse
			pin_state2&=~2;
		}
		if (pin_set2&2) {
			DDRA&=~(PIN_PIOB2); //Eingang
			PORTA|=(PIN_PIOB2); //Pullup
			pin_state2|=8;
			} else {
			DDRA|=(PIN_PIOB2); //Ausgang
			PORTA&=~(PIN_PIOB2); //Gegen masse
			pin_state2&=~8;
		}
#ifdef FHEM_PLATINE
		if (LPORT_CH2&LPIN_CH2) {			_delay_ms(50);			LPORT_CH2&=~LPIN_CH2;		}
		if (LPORT_CH1&LPIN_CH1) {			_delay_ms(50);			LPORT_CH1&=~LPIN_CH1;		}
#endif
#ifdef W1DAQ
		if ((LPORT_CH2&LPIN_CH2)==0) {			_delay_ms(50);			LPORT_CH2|=LPIN_CH2;		}
#endif
		OWST_MAIN_END 
	}


}

