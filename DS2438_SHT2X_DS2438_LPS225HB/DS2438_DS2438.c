// Copyright (c) 2018, Tobias Mueller tm(at)tm3d.de
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

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>  
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include "../common/I2C/USI_TWI_Master.h"
#include "../common/I2C/LPS225HB.h"
#include "../common/I2C/SHT2xV2.h"
#include "../common/calibr.h"

extern void OWINIT(void);
extern void EXTERN_SLEEP(void);



volatile uint8_t owid1[8]={0x26, 0x3B, 0xDA, 0x84, 0x00, 0x00, 0x03, 0xA2};/**/
volatile uint8_t owid2[8]={0x26, 0x3C, 0xDA, 0x84, 0x00, 0x00, 0x03, 0x27};/**/
volatile uint8_t config_info1[26]={0x02,23, 0x05,0x08, 0x2,20, 0x00,0x00, 0x02,29,0x00,29,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
volatile uint8_t config_info2[26]={0x01,0x06, 0x05,0x08, 0x04,0x07, 0,0, 0x02,0x07,0x00,0x07,0,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
	
#if (owid>128) 
#error "Variable not correct"
#endif

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;
volatile uint8_t wdcounter=1;


typedef union {
	#if  defined(__AVR_ATtiny25__)
	volatile uint8_t bytes[16];
	#else
	volatile uint8_t bytes[64];
	#endif
	struct {
		uint8_t status;  //1
		int16_t temp;  //2
		uint16_t voltage;  //4
		int16_t current;  //6
		uint8_t threshold; //8
		union {
		uint8_t page1[8]; //9
		struct {
			uint32_t etm;
			uint8_t ica;
			uint16_t offset;
			uint8_t f1;
		};
		};
		#if  defined(__AVR_ATtiny25__)
		#else
		union {
				uint8_t page2[8]; //17
				struct  {
				uint32_t dis; 
				uint32_t eoc;
				};
				};
			
		uint8_t page3[8]; //25
		
		uint8_t page4[8];  //33
		uint8_t page5[8];  //41
		uint8_t page6[8];  //49
		uint8_t page7[8];  //57
		
		#endif
	};
} pack1_t;
volatile pack1_t pack1;




typedef union {
	#if  defined(__AVR_ATtiny25__)
	volatile uint8_t bytes[16];
	#else
	volatile uint8_t bytes[64];
	#endif
	struct {
		uint8_t status;  //1
		int16_t temp;  //2
		uint16_t voltage;  //4
		int16_t current;  //6
		uint8_t threshold; //8
		
		uint8_t page1[8]; //9
		#if  defined(__AVR_ATtiny25__)
		#else
		uint8_t page2[8]; //17
		uint8_t page3[8]; //25
		
		uint8_t page4[8];  //33
		uint8_t page5[8];  //41
		uint8_t page6[8];  //49
		uint8_t page7[8];  //57
		
		#endif
	};
} pack2_t;
volatile pack2_t pack2;


#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
#else
ISR(WDT_vect) {
#endif
	wdcounter++;
	if (reset_indicator==1) reset_indicator++;
	else if (reset_indicator==2) mode=0;
}



volatile int16_t DS2438_1_TEMP;
volatile uint16_t DS2438_1_VAD;
volatile int16_t DS2438_2_TEMP;
volatile uint16_t DS2438_2_VAD;


uint8_t userRegister[1];
int16_t sRH,sT;
double temperatureC,humidityRH,hhum;
double l;
uint32_t P;
int16_t t;

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
	return (r==0);  //Offen mal HIH4030
	 
	 
}


int main(void){
    PRR|=(1<<PRADC);  // adc for save Power
	PORTA=0xFF;
	PORTB=0xFF-(1<<PORTB0); //Schalter kann gegen Masse sein und zieht dann immer Strom
	DDRB|=(1<<PORTB0); //Als Ausgang und 0
	OWINIT();
	DDRB|=(1<<PINB1);//Ausgang und 1
	DDRA|=(1<<PINA0);
	
	WDTCSR |= ((1<<WDCE) );   // Enable the WD Change Bit//| (1<<WDE)
	WDTCSR |=   (1<<WDIE) |              // Enable WDT Interrupt
	(1<<WDP3) | (1<<WDP0);   // Set Timeout to ~8 seconds
	
	MCUSR=0;

	pack2.page3[0]=0xF1; //Luftfeuchte
	pack1.page3[0]=0xF4; //Spannung


	if (testSW()) {
		config_info2[5]=12;
	}else{
		config_info2[5]=7;
	}

	USI_TWI_Master_Initialise();
	initSHT2x();
	_delay_ms(10);
	 LPS225HB_Init();
	gcontrol=1;
	sei();
    while(1)   {
		if (gcontrol) {
			//wdcounter=3;
			gcontrol=0;
				
		}
		if (wdcounter>3) {  
			getSHT2xHumTemp(&temperatureC,&humidityRH);
			double RH=calibr_hum(temperatureC,-0.2,humidityRH)*10.0;
			double TC =temperatureC *10.0-2;


			if (testSW()) {
				 DS2438_2_VAD=RH;
				 DS2438_2_TEMP=TC*25.6;
				 
				 config_info2[5]=12;	//10V = 100%
			}else{
				hhum=(1.0546-0.000216*TC)*(RH);
				//am2302_hum=0.318*hhum +76.0;
				DS2438_2_VAD=0.31*hhum +80;
				DS2438_2_TEMP=TC*25.6;
				config_info2[5]=7;
			}
			//DS2438_1_TEMP=DS2438_2_TEMP;

			LPS225HB_Readi(&t,&P) ;
			P=P-700*(uint32_t)(4096);
			P=((double)P/102.4);
			//cli();pack.A=ADC;sei();
			//cli();DS2438_1_TEMP=P;sei();
			DS2438_1_TEMP=P;
			P=P/20;
			DS2438_1_VAD=P;

		//	gcontrol=1;
			wdcounter=0;
			
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
		asm("SLEEP");
   }


}
