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
#include "../common/I2C/BMP280.h"
#include "../common/owSlave_tools.h"


#define OWST_EXTERN_VARS




uint8_t owid[8]={0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x40};/**/
uint8_t config_info[26]={0x02,0x03, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x02,14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	
#if (owid>128) 
#error "Variable not correct"
#endif






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








int main(void){
    OWST_INIT_USI_ON
	
	pack.temp=0x0550;
	pack.config=0x7F;
	pack.TH=75;
	pack.TL=70;
	pack.rrFF=0xFF;
	pack.rr00=0;
	pack.rr10=0x10;
	
	OWINIT();

	OWST_EN_PULLUP
	USI_TWI_Master_Initialise();
	_delay_ms(500);
	bmp280Init();
	_delay_ms(50);
	bmp280Init();
	//DDRB|=(1<<PINB1);
	gcontrol=1;
	sei();
    while(1)   {
	
		if (gcontrol) {
			//PORTB|=(1<<PINB1); //Dauer 5.4ms
			uint32_t P;
			int32_t t;
			bmp280ConvertInt(&t,&P,1);
			P=P-70000;
			P=P/20;
			uint16_t w=P;
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

		OWST_MAIN_END

   }


}