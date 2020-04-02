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
#include "../common/owSlave_tools.h"
#include "../common/I2C/USI_TWI_Master.h"
#include "../common/I2C/CDM7160.h"


OWST_EXTERN_VARS

uint8_t owid[8]={0x28, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x77, 0x6B};/**/
uint8_t config_info[26]={10,21, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x02,16,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	

OWST_WDT_ISR

typedef union {
	volatile uint8_t bytes[8];
	struct {
		int16_t temp;  //0
		uint8_t TH;  //2
		uint8_t TL;  //3
		uint8_t config;  //4
		uint8_t rrFF; //5
		uint8_t rr00; //6
		uint8_t rr10; //7
	};
} pack_t;
volatile pack_t pack;








int main(void) {
	OWST_INIT_USI_ON


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
	PORTB&=~(1<<PINB1);
	DDRB|=(1<<PINB1);
	PORTA&=~(1<<PINA0);

	OWST_EN_PULLUP
	OWST_WDR_CONFIG8

	_delay_ms(100);
	USI_TWI_Master_Initialise();
	CDM7160softReset();
	_delay_ms(200);
	
	CDM7160setMode(0); //Power Down Mode
	_delay_ms(200);
	CDM7160setAvCount(0x3F);
	CDM7160setFMode(1);
	pack.temp=CDM7160getCO2();
	CDM7160setMode(1);
	_delay_ms(200);
	gcontrol=1;
	//DDRB|=(1<<PINB1);
	//while (1) 			{volatile double l=CDM7160getCO2();}

	sei();
	#define PBR 1
    while(1)   {
	
		if (gcontrol) {
			if (wdcounter<PBR)	wdcounter=PBR;
			gcontrol=0;
		}
		if ( wdcounter==PBR) {
			//CDM7160setMode(1);
		}
		if (wdcounter>(PBR+1)) {
			wdcounter=0;
			//PORTB|=(1<<PINB1); //Dauer 2.3ms
			volatile int16_t l=(int16_t)CDM7160getCO2()-1280;	
			int16_t w=l;
			int8_t t8=w/16;
			int8_t af=0;
			if (t8>pack.TH) af=1;
			if (t8<=pack.TL) af=1; 
			cli();
			pack.temp=w;
			//pack.temp++;
			alarmflag=af;
			sei();	
			//CDM7160setMode(0);
			EXTERN_SLEEP();		
			//PORTB&=~(1<<PINB1);
					
		}

		
		OWST_MAIN_END
   }


}