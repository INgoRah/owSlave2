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


extern void OWINIT();
extern void EXTERN_SLEEP();

uint8_t owid[8]={0x26, 0xA3, 0xD9, 0x89, 0xDD, 0xDD, 0x05, 0x64};/**/
uint8_t config_info[26]={0x01,0x06, 0x05,0x08, 0x06,0x08, 0x00,0x00, 0x02,0x09,0x00,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};



extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;

volatile uint8_t wdcounter;


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


volatile int16_t ds_temp;
volatile uint16_t ds_vad;
volatile uint16_t ds_vdd;

#define CH1 (1<<PORTA0)
#define CH2 (1<<PORTA1)
#define CH3 (1<<PORTA2)
#define PIN_CH1 PINA
#define PIN_CH2 PINA
#define PIN_CH3 PINA

volatile uint8_t ch1,ch2,ch3;


#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
	#else
ISR(WDT_vect) {
	#endif
	wdcounter++;if (wdcounter==11) wdcounter=10;
	if (ch1>1) ch1++;
	if (ch2>1) ch2++;
	if (ch3>1) ch3++;
	if (reset_indicator==1) reset_indicator++;
	else if (reset_indicator==2) mode=0;

}
	
ISR(PCINT0_vect) {

				if ((PIN_CH1&CH1)==0) {if (ch1==0) ch1=1;} else {ds_temp=25600;ch1=0;}
				if ((PIN_CH2&CH2)==0) {if (ch2==0) ch2=1;} else {ds_vdd=0x1f4;ch2=0;}
				if ((PIN_CH3&CH3)==0) {if (ch3==0) ch3=1;} else {ds_vad=0x1f4;ch3=0;}
	
	wdcounter=10;
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
		PRR|=(1<<PRUSI);   //  usi for save Power
		PORTA=0xFF;  
		PORTB=0xFF-(1<<PORTB0); //Schalter kann gegen Masse sein und zieht dann immer Strom
		DDRB|=(1<<PORTB0); //Als Ausgang und 0
		
		
		OWINIT();

		ACSR|=(1<<ACD);  //Disable Comparator
		//ADCSRB|=(1<<ACME); //Disable Analog multiplexer
		MCUCR &=~(1<<PUD); //All Pins Pullup...
		
		// Set up Watch Dog Timer for Inactivity
		WDTCSR |= (1<<WDCE) ;   // Enable the WD Change Bit
		WDTCSR =   (1<<WDIE) |              // Enable WDT Interrupt
			(1<<WDP2) | (1<<WDP1);   // Set Timeout to ~2 seconds		
		
		GIMSK|=(1<<PCIE0);
		PCMSK0=(CH1)|(CH2)|(CH3);

		
		sei();
		
		while(1)   {
				if ((PIN_CH1&CH1)==0) {if (ch1==0) ch1=1;} else {ds_temp=25600;ch1=0;}
				if ((PIN_CH2&CH2)==0){if (ch2==0) ch2=1;} else {ds_vdd=0x1f4;ch2=0;}
				if ((PIN_CH3&CH3)==0) {if (ch3==0) ch3=1;} else {ds_vad=0x1f4;ch3=0;}

				if (ch1==10) {ch1=0;ds_temp=0;};
				if (ch2==10) {ch2=0;ds_vdd=0;};
				if (ch3==10) {ch3=0;ds_vad=0;};

				if ((gcontrol&1)==1) {
					if (ch1==1) {
						ch1=2;
					}
				}
				if ((gcontrol&2)==2) {
					if ((ch2==1)) {
						ch2=2;
					}
				}
				if ((gcontrol&4)==4) {
					if (ch3==1) {
						ch3=2;
					}
				}
				gcontrol=0;
	

	#define TEST_TIMER  ((TIMSK0 & (1<<TOIE0))==0)
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