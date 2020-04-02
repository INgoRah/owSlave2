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
#include "../common/I2C/TWI_Master.h"
#include "../common/I2C/CDM7160.h"
#include "../common/I2C/BMP280.h"
#include "../common/owSlave_tools.h"

OWST_EXTERN_VARS


uint8_t owid1[8]={0x28, 0xA3, 0xD9, 0x84, 0x00, 0x26, 0x05, 0x35};/**/
uint8_t owid2[8]={0x28, 0xA4, 0xD9, 0x84, 0x00, 0x26, 0x05, 0xB0};/**/

uint8_t config_info1[26]={10,21,0x00,0x00, 0x00,0x00, 0x00,0x00, 0x02,16,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t config_info2[26]={0x02,0x03,0x00,0x00, 0x00,0x00, 0x00,0x00, 0x02,14,0x00,0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	

OWST_WDT_ISR

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
} pack1_t;
volatile pack1_t pack1,pack2;









OWST_TESTSW


uint8_t userRegister[1];
uint16_t CO2,BMP;

int main(void){
	 OWST_INIT_USI_ON
   
	pack1.temp=0x0550;
	pack1.config=0x7F;
	pack1.TH=75;
	pack1.TL=70;
	pack1.rrFF=0xFF;
	pack1.rr00=0;
	pack1.rr10=0x10;
	PORTA=0xFF;
	PORTB=0xFF;
	OWINIT();
	OWST_EN_PULLUP

	OWST_WDR_CONFIG8
	
	if (testSW()) {
		config_info2[5]=12;
		}else{
		config_info2[5]=7;
	}

	MCUSR=0;
	TWI_Master_Initialise();
		bmp280Init();
		_delay_ms(50);
		bmp280Init();

	CDM7160softReset();
	_delay_ms(200);
	
	CDM7160setMode(0); //Power Down Mode
	_delay_ms(200);
	CDM7160setAvCount(0x3F);
	CDM7160setFMode(1);
	CO2=CDM7160getCO2();
	CDM7160setMode(1);
	_delay_ms(200);
	gcontrol=1;
	_delay_ms(10);
	gcontrol=1;
	sei();
    while(1)   {
		 
		if (gcontrol) {
			wdcounter=3;
			
		}

		if (wdcounter>2) {
				volatile int16_t l=(int16_t)CDM7160getCO2()-1280;

				uint16_t w1=l;
				uint32_t P;
				int32_t t;
				bmp280ConvertInt(&t,&P,1);
				P=P-70000;
				P=P/20;
				uint16_t w2=P;
				uint8_t t81=w1>>4;
				uint8_t af1=0;
				if (t81>pack1.TH) af1=1;
				if (t81<=pack1.TL) af1=1;

				cli();
				pack1.temp=w1;
				//pack.temp++;
				alarmflag=af1;
				sei();


		
				cli();
				pack2.temp=w2;
				sei();
				//pack.temp++;
				gcontrol=0;

				

			wdcounter=0;
		}
	
		if (gcontrol) {
			gcontrol=0;
			EXTERN_SLEEP();
			
		}

		OWST_MAIN_END	
   }


}