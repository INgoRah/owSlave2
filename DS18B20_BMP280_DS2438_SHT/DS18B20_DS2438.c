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
#include "../common/I2C/SHT2xV2.h"
#include "../common/I2C/BMP280.h"
#include "../common/calibr.h"
#include "../common/owSlave_tools.h"

OWST_EXTERN_VARS


uint8_t owid1[8]={0x28, 0xA3, 0xD9, 0x84, 0x00, 0x16, 0x05, 0x18};/**/
uint8_t owid2[8]={0x26, 0xA3, 0xD9, 0x84, 0x00, 0x16, 0x05, 0x67};/**/

uint8_t config_info1[26]={0x02,0x03, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x02,14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t config_info2[26]={0x01,0x06, 0x05,0x08, 0x04,0x07, 0x00,0x00, 0x02,7,0x00,7,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	

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
		uint16_t current;  //6
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
		//uint8_t crc;  //65
		#endif
	};
} pack2_t;
volatile pack2_t pack2;




volatile int16_t DS2438_2_TEMP;
volatile uint16_t DS2438_2_VAD;
volatile uint16_t DS2438_2_VDD=0x01F4;

OWST_TESTSW


uint8_t userRegister[1];
int16_t sRH,sT;
double temperatureC,humidityRH;
volatile double l;
	uint32_t P;
	int32_t t;

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
	pack2.page3[0]=0xF1;

	MCUSR=0;
	TWI_Master_Initialise();
	
	initSHT2x();
	_delay_ms(10);
	bmp280Init();
	gcontrol=1;
	sei();
    while(1)   {
		 
		if (gcontrol) {
			wdcounter=2;
			
		}

		if (wdcounter>2) {
				getSHT2xHumTemp(&temperatureC,&humidityRH);
				double RH=calibr_hum(temperatureC,-0.2,humidityRH)*10.0;
				double TC =temperatureC *10.0-2;

			 if (testSW()) {
					DS2438_2_VAD=RH;
					DS2438_2_TEMP=(double)TC*25.6;
				 //am2302_temp=am2302_temp-20;
				 config_info2[5]=12;
				 DS2438_2_VDD=0x01F4;
				 
				 }else{
				 
				 double hhum=(1.0546-0.000216*TC)*(RH);
				 //am2302_hum=0.318*hhum +76.0;
				 DS2438_2_VAD=0.31*hhum +80;
				 DS2438_2_TEMP=TC*25.6;
				 //am2302_temp=am2302_temp-20;
				 config_info2[5]=7;
				 DS2438_2_VDD=0x01F4;
			 }
			wdcounter=0;
		}
	
		if ((gcontrol&1)==1) {
			
				bmp280ConvertInt(&t,&P,1);
				P=P-70000;
				P=P/20;
				uint16_t w=P;
				uint8_t t8=w>>4;
				uint8_t af=0;
				if (t8>pack1.TH) af=1;
				if (t8<=pack1.TL) af=1;
				cli();
				pack1.temp=w;
				//pack.temp++;
				alarmflag=af;
				sei();
				gcontrol=0;
				EXTERN_SLEEP();	
		}
		if (gcontrol) {
			gcontrol=0;
			
		}

		OWST_MAIN_END	
   }


}