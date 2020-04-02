
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
#include "../common//I2C/MAX1164x.h"


extern void OWINIT();

extern void EXTERN_SLEEP();


uint8_t owid[8]={0x28, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x77, 0x6B};/**/
uint8_t config_info[26]={0x08,0x01, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x02,17,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	
#if (owid>128) 
#error "Variable not correct"
#endif

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;


uint8_t max_adr=0;

typedef union {
	volatile uint8_t bytes[8];
	struct {
		uint16_t temp;  //0
		uint8_t TH;  //2
		uint8_t TL;  //3
		uint8_t config;  //4
		uint8_t rrFF; //5
		uint8_t rr00; //6
		uint8_t rr10; //7
	};
} pack_t;
volatile pack_t pack;

volatile uint8_t wdcounter;

#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
#else
ISR(WDT_vect) {
#endif 
	wdcounter++;
	if (reset_indicator==1) reset_indicator++;
	else if (reset_indicator==2) mode=0;
}




#define CH0_M MAX1164x_C_SCAN0|MAX1164x_C_SGL
#define CH1_M MAX1164x_C_SCAN0|MAX1164x_C_SGL|MAX1164x_C_CS0
#define CH0_CH1 MAX1164x_C_SCAN0



int main(void){
    PRR|=(1<<PRADC);  // adc for save Power
	pack.temp=0x0550;
	pack.config=0x7F;
	pack.TH=75;
	pack.TL=70;
	pack.rrFF=0xFF;
	pack.rr00=0;
	pack.rr10=0x10;
	PORTA=0xFF;
	PORTB=0xFF;
	OWINIT();
	PORTB|=(1<<PINB1);
	DDRB|=(1<<PINB1);
	PORTA|=(1<<PINA0);
	DDRA|=(1<<PINA0);
	

	MCUCR &=~(1<<PUD); //All Pins Pullup...
	MCUCR |=(1<<BODS);
	
	WDTCSR |= ((1<<WDCE) );   // Enable the WD Change Bit//| (1<<WDE)
	WDTCSR |=   (1<<WDIE) |              // Enable WDT Interrupt
	(1<<WDP2) | (1<<WDP1)| (1<<WDP0);   // Set Timeout to ~2 seconds
	

	MCUSR=0;
	USI_TWI_Master_Initialise();
	MAX1164x_config(MAX1164x_S_SEL2|MAX1164x_S_SEL0,CH0_M);//#define MAX1164x_C_CS0
	_delay_ms(30); //Internal Referenz start
	//2970 -> 1,5V
	
	gcontrol=1;
	//DDRB|=(1<<PINB1);
	volatile double l;
	sei();
    while(1)   {
		if (wdcounter>0) {
			PORTB&=~(1<<PINB1);
			_delay_us(500);
			l=MAX1164x_read();
			PORTB|=(1<<PINB1);
			wdcounter=0;
		}
	
		if (gcontrol) {
			//PORTB|=(1<<PINB1); //Dauer 2.3ms
			//=MAX44009getlux(max_adr);		
			//if (l<0.030) l=0.030; //Darf nicht 0 sein. minimum -35°C Sensor minimum 0.045
			//double l=1000;
			
			uint16_t w=l;
			uint8_t t8=w>>4;
			uint8_t af=0;
			if (t8>pack.TH) af=1;
			if (t8<=pack.TL) af=1; 
			cli();
			pack.temp=w;
			//pack.temp++;
			alarmflag=af;
			sei();	
			EXTERN_SLEEP();		
			//PORTB&=~(1<<PINB1);
		}

		
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
		//MCUCR&=~(1<<ISC01);
		asm("SLEEP");
   }


}