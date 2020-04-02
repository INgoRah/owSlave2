
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



uint8_t owid[8]={0x28, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x02, 0xAC};/**/
uint8_t config_info[26]={0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x02,6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	
#if (owid>128) 
#error "Variable not correct"
#endif

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;


volatile uint8_t wdcounter;


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




#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
#else
ISR(WDT_vect) {
#endif 
	//sleep_disable();          // Disable Sleep on Wakeup
	wdcounter++;
	if (reset_indicator==1) reset_indicator++;
	else if (reset_indicator==2) mode=0;
/*	if (timeout==2) {
		DIS_TIMER;
		EN_OWINT;
		mode=OWM_SLEEP;
	}
	timeout++;*/
	//sleep_enable();           // Enable Sleep Mode

}


volatile double V,ktemp;

uint16_t ADmess() {
	 ADMUX=0b00101100;  //3V  ADC2+  ADC1- 1x
	 ADCSRA|=(1<<ADSC);
	 while ((ADCSRA&(1<<ADSC)));
	return ADC;
}

int main(void){
    //PRR|=(1<<PRUSI)|(1<<PRADC);  //Switch off usi and adc for save Power
	pack.temp=0x0550;
	pack.config=0x7F;
	pack.TH=75;
	pack.TL=70;
	pack.rrFF=0xFF;
	pack.rr00=0;
	pack.rr10=0x10;
	PORTA=0xFF-(1<<PINA1)-(1<<PINA2);
	PORTB=0xFF;
	OWINIT();

	MCUCR &=~(1<<PUD); //All Pins Pullup...
	MCUCR |=(1<<BODS);

	WDTCSR |= ((1<<WDCE) );   // Enable the WD Change Bit//| (1<<WDE)
	WDTCSR |=   (1<<WDIE) |              // Enable WDT Interrupt
	(1<<WDP2) | (1<<WDP1);   // Set Timeout to ~1 seconds
	MCUSR=0;
	sei();
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);//|(1<<ADPS0);
	
	
	uint16_t ares[16],sum;
	uint8_t par=0;
	sum=0;
	for(uint8_t i=0;i<16;i++) {
		//sum+=ares[i];
		sum+=ADmess();
	}
	par=0;
	wdcounter=0;
	gcontrol=1;

    while(1)   {
		if (wdcounter>0) {
//			ares[par]=ADmess();
			par++;
			if (par>15) par=0;
			wdcounter=0;
			sum=0;
			for(uint8_t i=0;i<16;i++) {
				//sum+=ares[i];
				sum+=ADmess();
			}


		}
		if (gcontrol) {
			PORTB|=(1<<PORTB0);
			//V=sum/20.0/1024.0*1.12*1000.0/16.0;
			//V=sum/20.0/1024.0*1.01*1000.0/16.0;
			V=sum/1024.0*182-55*16-16;
			if (V>125*16) V=125*16;
			if (V<-55*16) V=-55*16;
		
			uint16_t htemp=V;
		
			uint8_t t8=pack.temp>>4;
			uint8_t af=0;
			if (t8>pack.TH) af=1;
			if (t8<=pack.TL) af=1;
			cli();
			pack.temp=htemp;
			alarmflag=af;
			sei();
			EXTERN_SLEEP();
			PORTB&=~(1<<PORTB0);
		}

		
#if  defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)  || defined(__AVR_ATtiny85__)
			if (((TIMSK & (1<<TOIE0))==0)&& (mode==0))
#endif			
#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) ||defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
			if (((TIMSK0 & (1<<TOIE0))==0)&& (mode==0))
#endif
			  {
//			CLKPR=(1<<CLKPCE);
	//		CLKPR=(1<<CLKPS2); /*0.5Mhz*/
//			PORTB&=~(1<<PINB1);
			MCUCR|=(1<<SE)|(1<<SM1);
			MCUCR&=~(1<<ISC01);
		} else {
			MCUCR|=(1<<SE);
			MCUCR&=~(1<<SM1);
		}
		asm("SLEEP");
   }


}