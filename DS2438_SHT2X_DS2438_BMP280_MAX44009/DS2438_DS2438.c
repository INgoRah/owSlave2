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

//!!!!!Max Program size 7551 Byte

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>  
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include "../common/I2C/USI_TWI_Master.h"
#include "../common/I2C/SHT2xV2.h"
#include "../common/calibr.h"
#include "../common/I2C/BMP280.h"
#include "../common/I2C/MAX44009.h"
#include "../common/owSlave_tools.h"


OWST_EXTERN_VARS



volatile uint8_t owid1[8]={0x26, 0x01, 0x01, 0x01, 0x00, 0x00, 0x03, 0xF6};/**/
volatile uint8_t owid2[8]={0x26, 0x02, 0x01, 0x01, 0x00, 0x00, 0x03, 0xAF};/**/
volatile uint8_t config_info1[26]={0x01,0x06, 0x05,0x08, 4,7, 0x00,0x00, 0x02,7,0x00,7,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
volatile uint8_t config_info2[26]={3,4, 0x05,0x08, 2,20, 0,0           , 0x02,15,0x00,14,0,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
	


typedef union {
	volatile uint8_t bytes[64];
	struct {
		uint8_t status;  //1
		int16_t temp;  //2
		uint16_t voltage;  //4
		int16_t current;  //6
		uint8_t threshold; //8
		uint8_t page1[8]; //9
		uint8_t page2[8]; //17
			
		uint8_t page3[8]; //25
		
		uint8_t page4[8];  //33
		uint8_t page5[8];  //41
		uint8_t page6[8];  //49
		uint8_t page7[8];  //57
		
	};
} pack1_t;
volatile pack1_t pack1;




typedef union {
	volatile uint8_t bytes[64];
	struct {
		uint8_t status;  //1
		int16_t temp;  //2
		uint16_t voltage;  //4
		int16_t current;  //6
		uint8_t threshold; //8
		
		uint8_t page1[8]; //9
		
		uint8_t page2[8]; //17
		uint8_t page3[8]; //25
		
		uint8_t page4[8];  //33
		uint8_t page5[8];  //41
		uint8_t page6[8]; //25
		uint8_t page7[8];  //57
		
	};
} pack2_t;
volatile pack2_t pack2;

OWST_WDT_ISR
OWST_TESTSW





volatile int16_t DS2438_1_TEMP;
volatile uint16_t DS2438_1_VAD;
volatile int16_t DS2438_2_TEMP;
volatile uint16_t DS2438_2_VAD;



double temperatureC,humidityRH,hhum;
double l;
uint32_t P;
int32_t t;
uint8_t max_adr=0;


int main(void){
	OWST_INIT_USI_ON
	OWINIT();
	OWST_WDR_CONFIG8
	OWST_EN_PULLUP
	
	pack1.page3[0]=0xF1;
	pack2.page3[0]=0xF4;


	USI_TWI_Master_Initialise();
	_delay_ms(10);
	bmp280Init();
	_delay_ms(10);
	initSHT2x();
	_delay_ms(100);
	if (checkMAX44009(0)) max_adr=0; else max_adr=1 ;
	gcontrol=1;
	sei();

    while(1)   {
		if (gcontrol) wdcounter=1;
		if (wdcounter>0) {  //8s
			getSHT2xHumTemp(&temperatureC,&humidityRH);
			double RH=humidityRH*10.0;
			double TC =temperatureC *10.0;
			if (testSW()) {
				DS2438_1_VAD=RH;
				DS2438_1_TEMP=TC*25.6;
				config_info1[5]=12;	//10V = 100%
			}else{
				hhum=(1.0546-0.000216*TC)*(RH);
				//am2302_hum=0.318*hhum +76.0;
				DS2438_1_VAD=0.31*hhum +80;
				DS2438_1_TEMP=TC*25.6;
				config_info1[5]=7;
			}
			wdcounter=0;
			bmp280ConvertInt(&t,&P,1);
			uint16_t Pu=P/50;
			Pu=Pu-1400;
			DS2438_2_VAD=Pu;
			l=MAX44009getlux(max_adr);
			if (l<0.030) l=0.030; //Darf nicht 0 sein. minimum -35°C Sensor minimum 0.045
			//double l=1000;
			l=log(l)*10*16;
			DS2438_2_TEMP=l;
			
		}
	
		if (gcontrol) {
			gcontrol=0;
		}

		
		OWST_MAIN_END
   }


}
