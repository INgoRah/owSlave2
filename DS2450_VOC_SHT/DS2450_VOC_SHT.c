
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
#include "../common/I2C/MAX1164x.h"
#include "../common/I2C/SHT2xV2.h"
#include "../common/calibr.h"

extern void OWINIT();
extern void EXTERN_SLEEP();

uint8_t owid[8]={0x20, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x02, 0x5D};/**/
uint8_t config_info[26]={0x01,14,0x04,0x08, 8,8,11,0x08,0x02,7,7,17,17,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

#if (owid>128) 
#error "Variable not correct"
#endif

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;
volatile uint8_t wdcounter=1;

#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
#else
ISR(WDT_vect) {
#endif
	wdcounter++;
	if (reset_indicator==1) reset_indicator++;
	else if (reset_indicator==2) mode=0;
}


typedef union {
	volatile uint8_t bytes[0x20];
	struct {
		//Page0
		uint16_t A;  //0
		uint16_t B;  //2
		uint16_t C;  //4
		uint16_t D;  //6
		//Page1
		uint8_t CSA1;
		uint8_t CSA2;
		uint8_t CSB1;
		uint8_t CSB2;
		uint8_t CSC1;
		uint8_t CSC2;
		uint8_t CSD1;
		uint8_t CSD2;
		//Page2
		uint8_t LA;
		uint8_t HA;
		uint8_t LB;
		uint8_t HB;
		uint8_t LC;
		uint8_t HC;
		uint8_t LD;
		uint8_t HD;
		//Page3
		uint8_t FC1;
		uint8_t FC2;
		uint8_t FC3;
		uint8_t FC4;
		uint8_t VCCP;
		uint8_t FC5;
		uint8_t FC6;
		uint8_t FC7;
		uint8_t convc1;
		uint8_t convc2;
		
		
	};
} pack_t;
volatile pack_t pack;





volatile int16_t am2302_temp;
volatile uint16_t am2302_hum;


uint8_t userRegister[1];
int16_t sRH,sT;
double temperatureC,humidityRH,hhum;
double l;
double RH;
double TC;


uint8_t max_adr=0;
#define CH0_M MAX1164x_C_SCAN0|MAX1164x_C_SGL
#define CH1_M MAX1164x_C_SCAN0|MAX1164x_C_SGL|MAX1164x_C_CS0
#define CH0_CH1 MAX1164x_C_SCAN0
//|MAX1164x_C_CS0

uint16_t weekmaxarr[8];

//Kompensieren der Abhänigkeit von RS/RO von Temperatur und Luftfeuchte
inline double interp(double t, double h) {
	double h2;
	double t2;
	h2=h*h;
	t2=t*t;
	return 4.76111e-9*h2*t2-3.96956e-7*h2*t+0.0000408889*h2-1.07132e-6*h*t2+0.000115968*h*t-0.0101333*h+0.000163806*t2-0.0241179*t+1.80591;
}

double R0;
uint16_t mr;
uint8_t startup=10;
double ip;


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

#define EEPROM_R0 0
#define EEPROM_R0d 2
#define EEPROM_R0w 4
#define EEPROM_dol 6


uint16_t readEEPROM(uint8_t addr,uint16_t def) {
	uint16_t hr;
	while(EECR & (1<<EEPE));
	EEARL=addr+1;
	EECR |= (1<<EERE);
	hr=EEDR;
	if (hr!=0xFF) {
		hr=hr<<8;
		while(EECR & (1<<EEPE));
		EEARL=addr;
		EECR |= (1<<EERE);
		hr|=EEDR;
		return hr;
	}
	return def;
}

void writeEEPROM(uint8_t addr,uint16_t val) {
	while(EECR & (1<<EEPE));
	EECR = (0<<EEPM1)|(0<<EEPM0);
	EEARL = addr;
	EEDR = val&0xFF;
	EECR |= (1<<EEMPE);
	EECR |= (1<<EEPE);
	while(EECR & (1<<EEPE));
	EECR = (0<<EEPM1)|(0<<EEPM0);
	EEARL = addr+1;
	EEDR = val>>8;
	EECR |= (1<<EEMPE);
	EECR |= (1<<EEPE);
}

uint16_t pR0;
uint16_t pVS;
uint8_t pcmode;
int16_t pip;
uint16_t ptol_s8;
uint16_t ptol_d;
uint16_t pr_day_max;
uint16_t pr_week_max;
uint16_t RA;

int main(void){
	pack.A=0;
	pack.B=0;
	pack.C=0;
	pack.D=0;
	pack.CSA1=0x08;
	pack.CSA2=0x8C;
	pack.CSB1=0x08;
	pack.CSB2=0x8C;
	pack.CSC1=0x08;
	pack.CSC2=0x8C;
	pack.CSD1=0x08;
	pack.CSD2=0x8C;
	pack.HA=0xFF;
	pack.LA=0x00;
	pack.HB=0xFF;
	pack.LB=0x00;
	pack.HC=0xFF;
	pack.LC=0x00;
	pack.HD=0xFF;
	pack.LD=0x00;
	pack.VCCP=0;

    PRR|=(1<<PRADC);  // adc for save Power
    PORTA=0xFF;
    PORTB=0xFF-(1<<PORTB0); //Schalter kann gegen Masse sein und zieht dann immer Strom
    DDRB|=(1<<PORTB0); //Als Ausgang und 0
    OWINIT();
    DDRB|=(1<<PINB1);//Ausgang und 1
    DDRA|=(1<<PINA0);
    
    WDTCSR |= ((1<<WDCE) );   // Enable the WD Change Bit//| (1<<WDE)
    WDTCSR |=   (1<<WDIE) |              // Enable WDT Interrupt
    //(1<<WDP0) |(1<<WDP2) | (1<<WDP1);    // Set Timeout to ~2 seconds
    (1<<WDP3) | (1<<WDP0);    // Set Timeout to ~8 seconds

    MCUSR=0;
	if (testSW()) {
			writeEEPROM(EEPROM_R0,0xFF);
			writeEEPROM(EEPROM_R0d,0xFF);  //Maximum des Tages
			writeEEPROM(EEPROM_R0w,0xFF); //Maximum der Letzten 7 Tage
			writeEEPROM(EEPROM_dol,0xFF); //Anzahl der Betriebstage

	}


	
	OWINIT();
	pcmode=0;
	pR0=readEEPROM(EEPROM_R0,1);
	R0=pR0/100.0;
	
	pr_day_max=readEEPROM(EEPROM_R0d,1);
	pr_week_max=readEEPROM(EEPROM_R0w,1);
	ptol_d=readEEPROM(EEPROM_dol,0);
	ptol_s8=0;  //Tag faengt mit Einschalten an
	for(uint8_t i=0;i<7;i++) {
		weekmaxarr[i]=pr_week_max;
	}

	USI_TWI_Master_Initialise();
	initSHT2x();
	_delay_ms(100);
	MAX1164x_config(MAX1164x_S_SEL2|MAX1164x_S_SEL0,CH0_M);//#define MAX1164x_C_CS0
	_delay_ms(30); //Internal Referenz start
	//2970 -> 1,5V
	gcontrol=1;
 
	sei();
	
	//DDRB|=(1<<PINB1);

    while(1)   {
		if (testSW()) {
			R0=1;
			pR0=0;
			writeEEPROM(EEPROM_R0,0);
		}


		if (wdcounter>0) {  //8s
			ptol_s8++;
			if (ptol_s8>(10000))  {//10800 ist theortisch der Tag aber meistens zu lang
				ptol_s8=0;
				ptol_d++;  //rund 180 Jahre :-)
				pr_week_max=0;
				weekmaxarr[7]=pr_day_max;
				for(uint8_t i=0;i<7;i++) {
					weekmaxarr[i]=weekmaxarr[i+1];
					//maximum of week
					if (weekmaxarr[i]>pr_week_max) pr_week_max=weekmaxarr[i];
				}
				if (ptol_d>7) {
					pR0=pr_week_max;
					} else {
					pR0=pr_day_max;
				}
				//R0=//R0-0.5*(pack2.R0/100-R0);
				R0=R0-(R0-pR0/100.0)*0.5	;
				pR0=R0*100;
				writeEEPROM(EEPROM_R0,pR0);
				writeEEPROM(EEPROM_R0d,pr_day_max);  //Maximum des Tages
				writeEEPROM(EEPROM_R0w,pr_week_max); //Maximum der Letzten 7 Tage
				writeEEPROM(EEPROM_dol,ptol_d); //Anzahl der Betriebstage
				pr_day_max=0;
			}
			if (startup!=0) startup--;
			getSHT2xHumTemp(&temperatureC,&humidityRH);
			ip=interp(temperatureC,humidityRH);
			pip=ip*1000;
			RH=calibr_hum(temperatureC,-0.2,humidityRH);
			TC =temperatureC -0.2;
			mr=0;
			//Kritische Sektion !___Ein Breakpoint in dieser Section kann den TGS8100 zerstoeren!___________________________
			PORTB&=~(1<<PINB1); //Auf 0 Ziehen
			_delay_us(150);
			mr+=MAX1164x_read();
			_delay_us(150);
			mr+=MAX1164x_read();
			PORTB|=(1<<PINB1);
			//ENDE Kritische Sektion !______________________________
			//l=mr/2.0*2.048/4096;
			// l maximal 2  mr max 4096  //mr 2V=8000
			
			if (pcmode) { //cmode=0 V 0..2 V cmode=1 V 1.5..3.5V
				//l+=1.5; //Spannung real
				mr+=6000;
			}
			//if (l>1.8) {
			if (mr>7200) {
				if (pcmode==0) {
					MAX1164x_config(MAX1164x_S_SEL2|MAX1164x_S_SEL0,CH0_CH1);
					pcmode=1;
				}
			}
			//if (l<1.6) {
			if (mr<6400) {
				if (pcmode==1) {
					MAX1164x_config(MAX1164x_S_SEL2|MAX1164x_S_SEL0,CH0_M);
					pcmode=0;
				}
				
				
			}
			pVS=mr*5/2;
			l=mr/4000.0;
			l=( 3/l- 1) *30;
			RA=l*100;
			
			l=l/ip;
			
			if (startup==0){
				if (l>R0) {
					R0=l;
					pR0=R0*100;
					writeEEPROM(EEPROM_R0,pR0);

				}
				if (l*100>pr_day_max) {
					pr_day_max=l*100;
				}
				l=exp((1-(l/R0))*6.05)-1;// exp((1-($val)/55)*5.75);  (5.75 geht über 125 6.05 geht bis 240... mittlere Linie im Datenblatt)
				//l=(l/R0*100.0);
				l=l*100; //fuer DS2450

			} 
			else l=0; //negative Werte am Anfang verhintern
			wdcounter=0;
			
		}


		if (gcontrol) {
			//PORTB|=(1<<PINB1);
			uint8_t bb=1;
			uint8_t bb1=1;
			for(uint8_t i=0;i<4;i++){
				if (pack.convc1&bb1) {
					if (pack.convc2&(bb)) {pack.bytes[i*2]=0;pack.bytes[i*2+1]=0;}
					bb=bb<<1;
					if (pack.convc2&(bb)) {pack.bytes[i*2]=0xFF;pack.bytes[i*2+1]=0xFF;}
					bb=bb<<1;
				} else bb=bb<<2;
				bb1=bb1<<1;				
			}
			//CHanel A
			if (pack.convc1&1) {
				uint16_t ct=(temperatureC*100.0)+32767;
				cli();pack.A=ct;sei();
				alarmflag=0;
				if (pack.CSA2&0x08)  //AEH
					if (pack.bytes[1]>pack.HA) {alarmflag=1;pack.CSA2|=0x20;}
				if (pack.CSA2&0x04)  //AEL
					if (pack.bytes[1]<pack.LA) {alarmflag=1;pack.CSA2|=0x10;}
			}

			if (pack.convc1&2) {
				cli();pack.B=humidityRH*100;sei();
				if (pack.CSB2&0x08)  //AEH
					if (pack.bytes[1]>pack.HB) {alarmflag=1;pack.CSB2|=0x20;}
				if (pack.CSB2&0x04)  //AEL
					if (pack.bytes[1]<pack.LB) {alarmflag=1;pack.CSB2|=0x10;}
			}

			if (pack.convc1&4) {
				cli();pack.C=l;sei();
				if (pack.CSC2&0x08)  //AEH
					if (pack.bytes[1]>pack.HC) {alarmflag=1;pack.CSC2|=0x20;}
				if (pack.CSC2&0x04)  //AEL
					if (pack.bytes[1]<pack.LC) {alarmflag=1;pack.CSC2|=0x10;}
			} 
			if (pack.convc1&8) {
				if (pack.CSD2&0x01)	ADMUX=3; else ADMUX=0x83;
				_delay_us(100);
				ADCSRA|=(1<<ADSC);
				while ((ADCSRA&(1<<ADSC)));
				cli();pack.D=RA;sei();
				if (pack.CSD2&0x08)  //AEH
					if (pack.bytes[1]>pack.HD) {alarmflag=1;pack.CSD2|=0x20;}
				if (pack.CSD2&0x04)  //AEL
					if (pack.bytes[1]<pack.LD) {alarmflag=1;pack.CSD2|=0x10;}
			}
			
			EXTERN_SLEEP();
			//PORTB&=~(1<<PINB1);
		}

		uint8_t bb=1;
		for(uint8_t i=0;i<4;i++) {
			if (pack.bytes[8+i*2]&0x80) {  //Chanel as output
				if (pack.bytes[8+i*2]&0x40) {
					DDRA|=bb;
				} else {
					DDRA&=~bb;
				}
			} else {
				DDRA&=~bb;
			}
			bb=bb*2;
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
	//	asm("SLEEP");
   }


}