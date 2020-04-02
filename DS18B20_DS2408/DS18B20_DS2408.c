
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

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include "../common/I2C/TWI_Master.h"
#include "../common/owSlave_tools.h"
#include "../common/I2C/SHT3x.h"
#include "../common/calibr.h"


#include <math.h>

extern void OWINIT();
extern void EXTERN_SLEEP();
extern uint8_t stat_to_sample;

uint8_t owid1[8]={0x28, 0xA3, 0xD9, 0x84, 0x00, 0x16, 0x05, 0x18};/**/
uint8_t owid2[8]={0x29, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x01, 0x73};/**/
uint8_t config_info1[26]={0x01,0x01, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x02,29,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t config_info2[26]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0,0,0,0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

OWST_EXTERN_VARS

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
} pack1_t;
volatile pack1_t pack1;

typedef union {
	volatile uint8_t bytes[0x20];
	struct {
		uint8_t PIO_Logic_State;     //		0088h
		uint8_t PIO_Output_Latch_State;
		uint8_t PIO_Activity_Latch_State;
		uint8_t Conditional_Search_Channel_Selection_Mask;
		uint8_t Conditional_Search_Channel_Polarity_Selection;
		uint8_t Status; //008D
		uint8_t FF1;
		uint8_t FF2;
		
	};
} pack2_t;
volatile pack2_t pack2;

uint8_t values[10];
uint8_t ap=1;

uint8_t crc8_f() {
	uint8_t lscrc=0x0;
	for(uint8_t i=0;i<5;i++) {
		uint8_t v=values[i];
		//if (v==0) v=0xFF;
		uint8_t bit=1;
		uint8_t lb;
		for(uint8_t j=0;j<8;j++) {
			if ((v&bit)==bit) lb=1; else lb=0;
			if ((lscrc&1)!=lb)	lscrc=(lscrc>>1)^0x8c; else	lscrc=(lscrc>>1);
			bit=bit*2;
		
		
		}
	}
	return lscrc;
}


//Umstellung

//rh=(T-25)*(0,01+0,00008*x)-2,0468+0,0367*x-0,0000015955*x*x
//d1 for 3V
#define d1 -39.7  
#define d2 0.01
inline uint16_t calcSHT75_T(double real_t) {
	return (real_t-d1)/d2;
}

inline uint16_t calcSHT75RH_lin(double real_RHlin) {
	return 11501.1-0.280297*sqrt(1667284153.0-7977500.0*real_RHlin);
}

inline double calcSHT75H_tcorr(double real_t,double real_RHtrue) {
	return real_RHtrue-(real_t-25)*(0.01+0.00008*calcSHT75RH_lin(real_RHtrue));

}


double T=20.0;
double RH=60;

int main(void){
	OWST_INIT_USI_ON;
	pack2.FF1=0xFF;
	pack2.FF2=0xFF;
	 //0x0E 0x19 0x48 0x00
	 if (RH<8) RH=8;
	 uint16_t lt=calcSHT75_T(T);
		double lfc=calcSHT75H_tcorr(T,RH);
	 uint16_t lf=calcSHT75RH_lin(lfc);
	values[0]=0x00;
	values[1]=lt&0xFF; if (values[1]==0) values[1]=1;
	values[2]=lt>>8; if (values[2]==0) values[2]=1;
	values[3]=lf&0xFF; if (values[3]==0) values[3]=1;
	values[4]=lf>>8; if (values[4]==0) values[4]=1;
	values[5]=0x5D; 
	values[6]=0x00;
	values[7]=0x00;
	values[5]=crc8_f();	
	
	OWINIT();

	TWI_Master_Initialise();
	initSHT3x(0);
	_delay_ms(100);


	getSHT3xHumTemp(0,&T,&RH);
	OWST_WDR_CONFIG8;
	sei();
	stat_to_sample=0x55;
	while (1) {
		//stat_to_sample=0;
		if (reset_indicator) {
		//	ap=0;
		//	stat_to_sample=0;
		//	reset_indicator=0;
		}
		if (wdcounter>3) {
			
			wdcounter=0;
			RH=RH+0.2;
			getSHT3xHumTemp(0,&T,&RH);
			//pack1.temp=T*16.0;
			lt=calcSHT75_T(T);
			lfc=calcSHT75H_tcorr(T,RH);
			lf=calcSHT75RH_lin(lfc);
		values[0]=0x00;
		values[1]=lt&0xFF; if (values[1]==0) values[1]=1;
		values[2]=lt>>8; if (values[2]==0) values[2]=1;
		values[3]=lf&0xFF; if (values[3]==0) values[3]=1;
		values[4]=lf>>8; if (values[4]==0) values[4]=1;
		 values[5]=crc8_f();
		 if (values[5]==0) values[3]=values[3]+1;  //CRC darf nicht 0 sein ... warum auch immer
		 values[5]=crc8_f();
		}
		pack2.Status|=0x80;
		if (gcontrol&1) {
			uint8_t bb=1;
			for(uint8_t i=0;i<8;i++) {
				if ((pack2.PIO_Logic_State&bb)!=(pack2.PIO_Output_Latch_State&bb)) pack2.PIO_Activity_Latch_State|=bb;
				bb=bb*2;
			}
            pack2.PIO_Logic_State=pack2.PIO_Output_Latch_State;
			gcontrol&=~0x01;
		}
		if (gcontrol&2) {
			pack2.PIO_Activity_Latch_State=0;
            gcontrol&=~0x02;
		}
		if (gcontrol&4) {
			stat_to_sample=values[ap];
			ap++;		
			if (ap>5) {
					ap=0;
			}
			gcontrol&=~0x04;
		} 		
		if (gcontrol&8) {
			ap=1;
			stat_to_sample=values[ap];
			ap++;
			//if (ap>5) ap=1;
			gcontrol&=~0x08;
		} 
		if ((gcontrol&16)==16) {
			
	
			uint16_t w=T*16.0;
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

		OWST_MAIN_END
	}



}
