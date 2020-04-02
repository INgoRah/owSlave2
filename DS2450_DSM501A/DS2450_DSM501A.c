
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

uint8_t owid[8]={0x20, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x02, 0x5D};/**/
uint8_t config_info[26]={0x9,13,0x06,0x09,0x06,0x013,0x06,0x013,0x02,20,20,20,20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#if (owid>128) 
#error "Variable not correct"
#endif

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;

volatile uint16_t ltime_list[32];
volatile uint16_t pcount_list[32];
volatile uint8_t wsp=0;
volatile uint16_t max_count;
volatile uint16_t max_ptime=0;
volatile uint16_t min_ptime=0;

uint16_t ds;



typedef union {
	volatile uint8_t bytes[0x20];
	struct {
		//Page0
		uint16_t A;  //0
		uint16_t B;  //2
		uint16_t C;  //4
		uint16_t D;  //6
		//Page1
		uint8_t CSA1;
		uint8_t CSA2;
		uint8_t CSB1;
		uint8_t CSB2;
		uint8_t CSC1;
		uint8_t CSC2;
		uint8_t CSD1;
		uint8_t CSD2;
		//Page2
		uint8_t LA;
		uint8_t HA;
		uint8_t LB;
		uint8_t HB;
		uint8_t LC;
		uint8_t HC;
		uint8_t LD;
		uint8_t HD;
		//Page3
		uint8_t FC1;
		uint8_t FC2;
		uint8_t FC3;
		uint8_t FC4;
		uint8_t VCCP;
		uint8_t FC5;
		uint8_t FC6;
		uint8_t FC7;
		uint8_t convc1;
		uint8_t convc2;
		
		
	};
} pack_t;
volatile pack_t pack;

volatile uint16_t count;
volatile uint16_t ltime;
volatile uint16_t startt;

#define SEC_TIME 31250+300

ISR(TIM1_OVF_vect){
	TCNT1=~(SEC_TIME);
	ltime_list[wsp]=ltime;
	pcount_list[wsp]=count;
	wsp++;
	if (wsp>=32) wsp=0;
	//if (count>max) max=count;
	//if (mess) 
	//count=(SEC_TIME)
	startt=~(SEC_TIME);
	count=0;
	ltime=0;
	
}
#define PIN_WIND (1<<PINA2)
#define PIN_REG PINA
volatile uint8_t istat;
volatile uint8_t mess=0;


ISR(PCINT0_vect) {
	if (((PIN_REG&PIN_WIND)==0)&&((istat&PIN_WIND)==PIN_WIND)) {	
	 startt=TCNT1;
	 mess=1;
	PORTB&=~(1<<PINB1);
	}
	if (((PIN_REG&PIN_WIND)==PIN_WIND)&&((istat&PIN_WIND)==0)) {
		uint16_t duration=(+TCNT1-startt);
		ltime+=duration;
		count++;
		if (min_ptime==0) min_ptime=duration;
		if (duration>max_ptime)max_ptime=duration;
		if (duration<min_ptime)min_ptime=duration;
		
		mess=0;
	}

	istat=PIN_REG;
	
}


int main(void){
	for(uint8_t i=0;i<16;i++) {ltime_list[i]=0;pcount_list[i]=0;}
	pack.A=0;
	pack.B=0;
	pack.C=0;
	pack.D=0;
	pack.CSA1=0x08;
	pack.CSA2=0x8C;
	pack.CSB1=0x08;
	pack.CSB2=0x8C;
	pack.CSC1=0x08;
	pack.CSC2=0x8C;
	pack.CSD1=0x08;
	pack.CSD2=0x8C;
	pack.HA=0xFF;
	pack.LA=0x00;
	pack.HB=0xFF;
	pack.LB=0x00;
	pack.HC=0xFF;
	pack.LC=0x00;
	pack.HD=0xFF;
	pack.LD=0x00;
	pack.VCCP=0;
	OWINIT();

	MCUCR &=~(1<<PUD); //All Pins Pullup...
	MCUCR |=(1<<BODS);
	PORTA&=~((1<<PINA0)|(1<<PINA1)|(1<<PINA2)|(1<<PINA3));
	ADCSRA=(1<<ADEN)|(1<ADPS0)|(1<<ADPS2);

	
	
	gcontrol=1;
	ADCSRB|=(1<<ADLAR); 
	sei();
	
	DDRB|=(1<<PINB1);
	TCCR1B|=(1<<CS12); //8000000/256 =31250
	TIMSK1|=(1<<TOIE1); 
	istat=PIN_REG;
	GIMSK|=(1<<PCIE0);
	PCMSK0=(PIN_WIND);
	MCUCR &=~(1<<PUD); //All Pins Pullup...
	PORTA|=0xFF; 
    while(1)   {
		//PORTB&=~(1<<PINB1);

		if (gcontrol) {
			//PORTB|=(1<<PINB1);
			uint8_t bb=1;
			uint8_t bb1=1;
			for(uint8_t i=0;i<4;i++){
				if (pack.convc1&bb1) {
					if (pack.convc2&(bb)) {pack.bytes[i*2]=0;pack.bytes[i*2+1]=0;}
					bb=bb<<1;
					if (pack.convc2&(bb)) {pack.bytes[i*2]=0xFF;pack.bytes[i*2+1]=0xFF;}
					bb=bb<<1;
				} else bb=bb<<2;
				bb1=bb1<<1;				
			}
			//CHanel A
			if (pack.convc1&1) {
				//if (pack.CSA2&0x01)	ADMUX=0; else ADMUX=0x80;
				//_delay_us(100);
				//ADCSRA|=(1<<ADSC);
				//while ((ADCSRA&(1<<ADSC)));
				//cli();pack.A=ADC;sei();
				ds=0;
				for(uint8_t i=0;i<32;i++) {
					ds+=pcount_list[i];
					//if (wspeed[i]>ds) ds=wspeed[i];
				}
				cli();pack.A=ds;sei();
				alarmflag=0;
				if (pack.CSA2&0x08)  //AEH
					if (pack.bytes[1]>pack.HA) {alarmflag=1;pack.CSA2|=0x20;}
				if (pack.CSA2&0x04)  //AEL
					if (pack.bytes[1]<pack.LA) {alarmflag=1;pack.CSA2|=0x10;}
			}

			if (pack.convc1&2) {
				double fds=0;
				for(uint8_t i=0;i<32;i++) {
					fds+=ltime_list[i];
					//if (wspeed[i]>ds) ds=wspeed[i];
				}
				double lr=(fds/32.0)/(SEC_TIME)*65535/5.1;
				cli();pack.B=lr;sei();
				//max=0;
				if (pack.CSB2&0x08)  //AEH
					if (pack.bytes[1]>pack.HB) {alarmflag=1;pack.CSB2|=0x20;}
				if (pack.CSB2&0x04)  //AEL
					if (pack.bytes[1]<pack.LB) {alarmflag=1;pack.CSB2|=0x10;}
			}

			if (pack.convc1&4) {
				double mu_sek=(double)max_ptime/(SEC_TIME)*1e6;
				max_ptime=0;
				cli();pack.C=mu_sek;sei();
				if (pack.CSC2&0x08)  //AEH
					if (pack.bytes[1]>pack.HC) {alarmflag=1;pack.CSC2|=0x20;}
				if (pack.CSC2&0x04)  //AEL
					if (pack.bytes[1]<pack.LC) {alarmflag=1;pack.CSC2|=0x10;}
			} 
			if (pack.convc1&8) {
				double mu_sek=(double)min_ptime/(SEC_TIME)*1e6;
				min_ptime=0;
				cli();pack.D=mu_sek;sei();
				if (pack.CSD2&0x08)  //AEH
					if (pack.bytes[1]>pack.HD) {alarmflag=1;pack.CSD2|=0x20;}
				if (pack.CSD2&0x04)  //AEL
					if (pack.bytes[1]<pack.LD) {alarmflag=1;pack.CSD2|=0x10;}
			}
			
			EXTERN_SLEEP();
			//
		}

		uint8_t bb=1;
		for(uint8_t i=0;i<4;i++) {
			if (pack.bytes[8+i*2]&0x80) {  //Chanel as output
				if (pack.bytes[8+i*2]&0x40) {
					DDRA|=bb;
				} else {
					DDRA&=~bb;
				}
			} else {
				DDRA&=~bb;
			}
			bb=bb*2;
		}
		if ((PORTB&(1<<PINB1))==0) {
			_delay_ms(100);
			PORTB|=(1<<PINB1);
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
	//	asm("SLEEP");
   }


}