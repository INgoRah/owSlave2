
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

#if defined(__AVR_ATtiny85__)
#define PCINT_VECTOR PCINT0_vect
#define PORT_REG PORTB
#define PIN_REG PINB
#define PIN_DDR DDRB

#define PIN_PIO0 (1<<PINB4)
#define PIN_PIO1 (1<<PINB3)
#define PIN_PIO2 (1<<PINB1)
#define PIN_PIO3 (1<<PINB0)
#define LED _BV(PB3)
#endif

#if  defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define PCINT_VECTOR PCINT0_vect
#define PIN_REG PINA
#define PORT_REG PORTA
#define PIN_DDR DDRA

#define PIN_PIO0 (1<<PINA1)
#define PIN_PIO1 (1<<PINA2)
#define PIN_PIO2 (1<<PINA3)
#define PIN_PIO3 (1<<PINA5)
#define PIN_PIO4 (1<<PINA6)
#define LED _BV(PA0)
#endif

#include <math.h>

extern void OWINIT(void);
extern void EXTERN_SLEEP(void);
extern uint8_t stat_to_sample;

uint8_t owid[8]={0x29, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x01, 0x73};/**/
uint8_t config_info[26]={0x06,0x09,0x06,0x09,0x06,0x09,0x06,0x09,0x02,20,20,20,20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

OWST_EXTERN_VARS

OWST_WDT_ISR

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
} pack_t;
volatile pack_t pack;

uint8_t values[10];
uint8_t ap=1;

uint8_t crc8() {
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
	pack.FF1=0xFF;
	pack.FF2=0xFF;
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
		values[1]=8;
		values[2]=26;
		values[3]=0;
		values[4]=5;
		values[5]=0x5D;
	values[6]=0x00;
	values[7]=0x00;
	values[5]=crc8();	
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
			lt=calcSHT75_T(T);
			lfc=calcSHT75H_tcorr(T,RH);
			lf=calcSHT75RH_lin(lfc);
		values[0]=0x00;
		values[1]=lt&0xFF; if (values[1]==0) values[1]=1;
		values[2]=lt>>8; if (values[2]==0) values[2]=1;
		values[3]=lf&0xFF; if (values[3]==0) values[3]=1;
		values[4]=lf>>8; if (values[4]==0) values[4]=1;
		values[1]=8;
		values[2]=26;
		values[3]=0;
		values[4]=5;
		values[5]=0x5D;
		 values[5]=crc8();
		}
		pack.Status|=0x80;
		if (gcontrol&1) {
			uint8_t bb=1;
			for(uint8_t i=0;i<8;i++) {
				if ((pack.PIO_Logic_State&bb)!=(pack.PIO_Output_Latch_State&bb)) pack.PIO_Activity_Latch_State|=bb;
				bb=bb*2;
			}
            pack.PIO_Logic_State=pack.PIO_Output_Latch_State;
			gcontrol&=~0x01;
		}
		if (gcontrol&2) {
			pack.PIO_Activity_Latch_State=0;
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

		OWST_MAIN_END
	}



}
