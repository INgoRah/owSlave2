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
#include "../common/I2C/TWI_Master.h"
#include "../common/I2C/BMP280.h"
#include "../common/I2C/CDM7160.h"
#include "../common/owSlave_tools.h"


OWST_EXTERN_VARS



volatile uint8_t owid1[8]={0x26, 0x01, 0x01, 0x01, 0x00, 0x00, 0x03, 0xF6};/**/
volatile uint8_t owid2[8]={0x26, 0x02, 0x01, 0x01, 0x00, 0x00, 0x03, 0xAF};/**/
volatile uint8_t config_info1[26]={1,6, 10,8, 10,8, 0x00,0x00, 0x02,14,16,16,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
volatile uint8_t config_info2[26]={1,6, 2,8, 2,8, 0,0           , 0x02,14,14,14,0,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
	


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
volatile uint16_t DS2438_1_VDD;
volatile int16_t DS2438_2_TEMP;
volatile uint16_t DS2438_2_VAD;
volatile uint16_t DS2438_2_VDD;



double temperatureC,humidityRH,hhum;
double l;
uint32_t P;
int32_t t;
uint8_t max_adr=0;
int16_t CO2;


int main(void){
	OWST_INIT_USI_ON
	OWINIT();
	OWST_WDR_CONFIG8
	OWST_EN_PULLUP
	
	pack1.page3[0]=0xF1;
	pack2.page3[0]=0xF4;

	TWI_Master_Initialise();
	CDM7160softReset();
	_delay_ms(200);

	CDM7160setMode(0); //Power Down Mode
	_delay_ms(200);
	CDM7160setAvCount(0x3F);
	CDM7160setFMode(1);
	CO2=CDM7160getCO2();
	CDM7160setMode(1);
	_delay_ms(200);

	bmp280Init();




	gcontrol=1;
	sei();

    while(1)   {
		if (gcontrol) wdcounter=4;
		if (wdcounter>3) {  //8s
			wdcounter=0;
			bmp280ConvertInt(&t,&P,1);
			_delay_ms(100);
			CO2=CDM7160getCO2()-35;
			double cc=(double)CO2/(P/101300.0*298.0/(273.0+t/100.0));
			DS2438_2_TEMP=t/100.0*256.0;
			DS2438_1_TEMP=CO2;
			DS2438_1_VDD=cc/10;
			DS2438_1_VAD=(cc-DS2438_1_VDD*10)*100;
			DS2438_2_VDD=P/1000;
			DS2438_2_VAD=(P-DS2438_2_VDD*1000);

			
		}
	
		if (gcontrol) {
			gcontrol=0;
		}

		
		OWST_MAIN_END
   }


}
