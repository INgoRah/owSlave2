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
#include "../common/owSlave_tools.h"


OWST_EXTERN_VARS
OWST_WDT_ISR


//#define W1DAQ
#define JOE_M
volatile uint8_t owid1[8]={0x26, 0x61, 0xDA, 0x84, 0x00, 0x00, 0x03, 0x43};/**/
volatile uint8_t owid2[8]={0x26, 0x62, 0xDA, 0x84, 0x00, 0x00, 0x03, 0x1A};/**/
volatile uint8_t config_info1[26]={6,6,6,0x08, 6,8, 0x00,0x00, 0x02,20,20,20,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
volatile uint8_t config_info2[26]={6,6, 6,0x08,6,8, 0,0, 0x02,20,20,20,0,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
	
#if (owid>128) 
#error "Variable not correct"
#endif

#ifdef JOE_M
#define PIN_PIOA1 (1<<PINA4)
#define ADMA1 PINA4
#define PIN_PIOB1 (1<<PINA5)
#define ADMB1 PINA5
#define PIN_PIOA2 (1<<PINA6)
#define ADMA2 PINA6
#define PIN_PIOB2 (1<<PINA7)
#define ADMB2 PINA7
#define ADDIFF1 0b011010
#define ADDIFF1G 0b011011
#define ADDIFF2 0b011110
#define ADDIFF2G 0b011111
#endif

#ifdef W1DAQ
#define PIN_PIOB1 (1<<PINA1)
#define ADMB1 PINA1
#define PIN_PIOA1 (1<<PINA0)
#define ADMA1 PINA0
#define PIN_PIOB2 (1<<PINA7)
#define ADMB2 PINA7
#define PIN_PIOA2 (1<<PINA3)
#define ADMA2 PINA3
#define ADDIFF1 0b001000
#define ADDIFF1G 0b001001
#define ADDIFF2 0b011000
#define ADDIFF2G 0b011001

#endif





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
} pack_t;
volatile pack_t pack2,pack1;



volatile int16_t DS2438_1_TEMP;
volatile uint16_t DS2438_1_VAD;
volatile uint16_t DS2438_1_VDD;
volatile int16_t DS2438_2_TEMP;
volatile uint16_t DS2438_2_VAD;
volatile uint16_t DS2438_2_VDD;

OWST_ADC_CONF16_FUNC
OWST_ADC_CONF16_OSS_FUNC
OWST_TESTSW


int main(void){
	 OWST_INIT_ADC_ON 
	pack2.page3[0]=0xF4;//Spannung
	pack1.page3[0]=0xF4; //Spannung
	OWINIT();
	OWST_WDR_CONFIG4
	OWST_EN_PULLUP
	
	PORTA&=~((PIN_PIOA1)|(PIN_PIOB1)|(PIN_PIOA2)|(PIN_PIOB2));
	OWST_INIT_ADC
	DIDR0=(PIN_PIOA1)|(PIN_PIOB1)|(PIN_PIOA2)|(PIN_PIOB2);

	//ADCSRB|=(1<<ADLAR);	Adiust left
	volatile double VCC;
	volatile double VAD_A,VAD_B,VAD_C,VAD_D;
	
	gcontrol=1;
	sei();
    while(1)   {
		if (gcontrol) {
			wdcounter=3;
			gcontrol=0;
				
		}
		if (wdcounter>2) { 
			
			wdcounter=0;
			ADMUX=OWST_ADCIN_REFINT;
			_delay_us(100);
			VCC=owst_ADC_runf();
			VCC=(1.079*65472.0)/VCC;
			DS2438_2_VDD=VCC*100;
			DS2438_1_VDD=VCC*100;

			if (testSW()) {
				ADMUX= ADDIFF1; //ADC0 + ADC1 - Gain 1
				_delay_us(100);
				VAD_A=owst_ADC_runf();
				if (VAD_A<3100) {
					ADMUX= ADDIFF1G; //ADC0 + ADC1 - Gain 20
					_delay_us(100);
					VAD_A=owst_ADC_runf();
					VAD_A=VCC/20.0*VAD_A/65472.0;
				} else {
					VAD_A=owst_ADC_OSS_runf();
					VAD_A=VCC*VAD_A/65472.0;
				}
				DS2438_1_VAD=VAD_A*100;
				DS2438_1_TEMP=VAD_A*256;

				ADMUX= ADDIFF2; //ADC0 + ADC1 - Gain 1
				_delay_us(100);
				VAD_B=owst_ADC_runf();
				if (VAD_B<3100) {
					ADMUX= ADDIFF2G; //ADC0 + ADC1 - Gain 20
					_delay_us(100);
					VAD_B=owst_ADC_runf();
					VAD_B=VCC/20.0*VAD_B/65472.0;
					} else {
					VAD_B=owst_ADC_OSS_runf();
					VAD_B=VCC*VAD_B/65472.0;
				}
				DS2438_2_VAD=VAD_B*100;
				DS2438_2_TEMP=VAD_B*256;



			} else {
				ADMUX=ADMA1;
				_delay_us(100);
				VAD_A=owst_ADC_OSS_runf();
				VAD_A=VCC*VAD_A/65472.0;
				DS2438_1_TEMP=VAD_A*256;
				
				ADMUX=ADMB1;
				_delay_us(100);
				VAD_B=owst_ADC_OSS_runf();
				VAD_B=VCC*VAD_B/65472.0;
				DS2438_1_VAD=VAD_B*100;
				
				ADMUX=ADMA2;
				_delay_us(100);
				VAD_C=owst_ADC_OSS_runf();
				VAD_C=VCC*VAD_C/65472.0;
				DS2438_2_TEMP=VAD_C*256;
				
				ADMUX=ADMB2;
				_delay_us(100);
				VAD_D=owst_ADC_OSS_runf();
				VAD_D=VCC*VAD_D/65472.0;
				DS2438_2_VAD=VAD_D*100;
			}
			
		

			
		}
	
		
		OWST_MAIN_END
   }


}
