
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
#include "../common/I2C/USI_TWI_Master.h"
#include "../common/I2C/MAX1164x.h"
#include "../common/I2C/SHT2x.h"
#include "../common/calibr.h"
#include "../common/I2C/BMP280.h"



extern void OWINIT();
extern void EXTERN_SLEEP();

uint8_t owid[8]={0x20, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x02, 0x5D};/**/
uint8_t config_info[26]={0x01,14, 0x04,0x08, 0x08,1, 0x02,16,0x02,7,7,17,14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

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

volatile uint16_t _temp;
volatile uint16_t _hum;


uint8_t userRegister[1];
int16_t sRH,sT;
double temperatureC,humidityRH,hhum;
double l;

double R0;
uint16_t mr;
uint8_t startup=10;
double ip;

uint32_t bmp_P;
int32_t bmp_t;

uint8_t max_adr=0;
#define CH0_M MAX1164x_C_SCAN0|MAX1164x_C_SGL
#define CH1_M MAX1164x_C_SCAN0|MAX1164x_C_SGL|MAX1164x_C_CS0
#define CH0_CH1 MAX1164x_C_SCAN0
//|MAX1164x_C_CS0

uint16_t weekmaxarr[8];


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


typedef struct {
	int16_t current;  //6
	uint16_t R0;
	//uint16_t VS;
	uint8_t cmode;
	int16_t ip;
	//uint8_t free;
	uint16_t tol_s8;
	uint16_t tol_d;
	uint16_t r_day_max;
	uint16_t r_week_max;
} pack2_t;
volatile pack2_t pack2;


//Kompensieren der Abhänigkeit von RS/RO von Temperatur und Luftfeuchte
inline double interp(double t, double h) {
	double h2;
	 double t2;
	h2=h*h;
	t2=t*t;
	return 4.76111e-9*h2*t2-3.96956e-7*h2*t+0.0000408889*h2-1.07132e-6*h*t2+0.000115968*h*t-0.0101333*h+0.000163806*t2-0.0241179*t+1.80591;
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





int main(void){
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

	pack2.cmode=0;
	pack2.R0=readEEPROM(EEPROM_R0,1);
	R0=pack2.R0/100.0;
		
	pack2.r_day_max=readEEPROM(EEPROM_R0d,1);
	pack2.r_week_max=readEEPROM(EEPROM_R0w,1);
	pack2.tol_d=readEEPROM(EEPROM_dol,0); 
	pack2.tol_s8=0;  //Tag faengt mit Einschalten an
	for(uint8_t i=0;i<7;i++) {
		weekmaxarr[i]=pack2.r_week_max;
	}

	USI_TWI_Master_Initialise();
	bmp280Init();
	SHT2x_SoftReset();
	SHT2x_ReadUserRegister(userRegister);
	SHT2x_WriteUserRegister(userRegister); //write changed user reg
	MAX1164x_config(MAX1164x_S_SEL2|MAX1164x_S_SEL0,CH0_M);//#define MAX1164x_C_CS0
	_delay_ms(30); //Internal Referenz start
	//2970 -> 1,5V	
	gcontrol=1;
	sei();
	

    while(1)   {

		/*if (pack1.config==0x1F) {
			pack1.config=0x7F;
			R0=1;
			pack2.R0=0;
			writeEEPROM(EEPROM_R0,0);
		} 
		if (pack1.config==0x05) {
			pack1.config=0x7F;				
			pack2.r_day_max=1;
			pack2.r_week_max=1;
			pack2.tol_d=0;
			pack2.tol_s8=0;  //Tag faengt mit Einschalten an	
			R0=1;	
			writeEEPROM(EEPROM_R0,0xFF);
			writeEEPROM(EEPROM_R0d,0xFF);  //Maximum des Tages
			writeEEPROM(EEPROM_R0w,0xFF); //Maximum der Letzten 7 Tage
			writeEEPROM(EEPROM_dol,0xFF); //Anzahl der Betriebstage		
			for(uint8_t i=0;i<7;i++) {
				weekmaxarr[i]=1;
			}				
		} */
		if (wdcounter>0) {
			pack2.tol_s8++;
			if (pack2.tol_s8>(10000))  {//10800 ist theortisch der Tag aber meistens zu lang
				pack2.tol_s8=0;
				pack2.tol_d++;  //rund 180 Jahre :-)
				pack2.r_week_max=0;
				weekmaxarr[7]=pack2.r_day_max;
				for(uint8_t i=0;i<7;i++) {
					weekmaxarr[i]=weekmaxarr[i+1];
					if (weekmaxarr[i]>pack2.r_week_max) pack2.r_week_max=weekmaxarr[i];
				}
				if (pack2.tol_d>7) {
					pack2.R0=pack2.r_week_max;
				} else {
					pack2.R0=pack2.r_day_max;
				}
				//R0=pack2.R0/100.0-0.5*(pack2.R0/100.0-R0);
				R0=R0-(R0-pack2.R0/100.0)*0.5	;
				pack2.R0=R0*100;
				writeEEPROM(EEPROM_R0,pack2.R0);
				writeEEPROM(EEPROM_R0d,pack2.r_day_max);  //Maximum des Tages
				writeEEPROM(EEPROM_R0w,pack2.r_week_max); //Maximum der Letzten 7 Tage
				writeEEPROM(EEPROM_dol,pack2.tol_d); //Anzahl der Betriebstage
				pack2.r_day_max=0;
			}
			if (startup!=0) startup--;
			SHT2x_MeasurePoll(HUMIDITY, &sRH);
			// --- measure temperature with "Polling Mode" (no hold master) ---
			SHT2x_MeasurePoll(TEMP, &sT);
			//-- calculate humidity and temperature --
			temperatureC = SHT2x_CalcTemperatureC(sT);
			humidityRH = SHT2x_CalcRH(sRH);
			ip=interp(temperatureC,humidityRH);
			pack2.ip=ip*1000;
			//humidityRH=calibr_hum(temperatureC,-0.45,humidityRH);
			//temperatureC =temperatureC -0.45;
			_hum= humidityRH*100.0;
			_temp=(temperatureC*100.0)+32767;
			//PORTB&=~(1<<PINB1);
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
			
			if (pack2.cmode) { //cmode=0 V 0..2 V cmode=1 V 1.5..3.5V
				//l+=1.5; //Spannung real
				mr+=6000;
			} 
			//if (l>1.8) {
			if (mr>7200) {				
				if (pack2.cmode==0) {
					MAX1164x_config(MAX1164x_S_SEL2|MAX1164x_S_SEL0,CH0_CH1);
					pack2.cmode=1;
				}
			}
			//if (l<1.6) {
			if (mr<6400) {	
				if (pack2.cmode==1) {
					MAX1164x_config(MAX1164x_S_SEL2|MAX1164x_S_SEL0,CH0_M);
					pack2.cmode=0;
				}
				
				
			}
			//pack2.VS=mr*5/2;
			l=mr/4000.0;
			l=( 3/l- 1) *30;
			pack2.current=l*100;
			
			l=l/ip;
			
			if (startup==0){
				if (l>R0) {
					R0=l;
					pack2.R0=R0*100;
					writeEEPROM(EEPROM_R0,pack2.R0);

				}
				if (l*100>pack2.r_day_max) {
					pack2.r_day_max=l*100;
				}
			} else if (l<R0) l=R0; //negative Werte am Anfang verhintern
			l=R0/l;
			l=log(l);
			l=l*160*4; //fuer DS18B20
			bmp280ConvertIntP1(&bmp_P);
			bmp_P=bmp_P/3;    // /100.0*32.0;
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
				cli();pack.A=_temp;sei();
				alarmflag=0;
				if (pack.CSA2&0x08)  //AEH
					if (pack.bytes[1]>pack.HA) {alarmflag=1;pack.CSA2|=0x20;}
				if (pack.CSA2&0x04)  //AEL
					if (pack.bytes[1]<pack.LA) {alarmflag=1;pack.CSA2|=0x10;}
			}

			if (pack.convc1&2) {
				cli();pack.B=_hum;sei();
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
				cli();pack.D=bmp_P;sei();
				if (pack.CSD2&0x08)  //AEH
					if (pack.bytes[1]>pack.HD) {alarmflag=1;pack.CSD2|=0x20;}
				if (pack.CSD2&0x04)  //AEL
					if (pack.bytes[1]<pack.LD) {alarmflag=1;pack.CSD2|=0x10;}
			}
			
			EXTERN_SLEEP();
			//PORTB&=~(1<<PINB1);
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
