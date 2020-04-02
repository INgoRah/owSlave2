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

extern void OWINIT(void);
extern void EXTERN_SLEEP(void);



volatile uint8_t owid1[8]={0x26, 0x3B, 0xDA, 0x84, 0x00, 0x00, 0x03, 0xA2};/**/
volatile uint8_t owid2[8]={0x26, 0x3C, 0xDA, 0x84, 0x00, 0x00, 0x03, 0x27};/**/
volatile uint8_t config_info1[26]={0x01,0x06, 0x05,0x08, 0x8,19, 0x00,0x00, 0x02,7,0x00,17,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
volatile uint8_t config_info2[26]={0x01,0x06, 0x05,0x08, 0x04,0x07, 11,0x08, 0x02,0x07,0x00,0x07,17,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	
	
#if (owid>128) 
#error "Variable not correct"
#endif

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;
volatile uint8_t wdcounter=1;
extern uint8_t cpsp2;


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
		int16_t current;  //6
		uint8_t threshold; //8
		union {
		uint8_t page1[8]; //9
		struct {
			uint32_t etm;
			uint8_t ica;
			uint16_t offset;
			uint8_t f1;
		};
		};
		#if  defined(__AVR_ATtiny25__)
		#else
		union {
				uint8_t page2[8]; //17
				struct  {
				uint32_t dis; 
				uint32_t eoc;
				};
				};
			
		uint8_t page3[8]; //25
		
		uint8_t page4[8];  //33
		uint8_t page5[8];  //41
		uint8_t page6[8];  //49
		uint8_t page7[8];  //57
		
		#endif
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
		int16_t current;  //6
		uint8_t threshold; //8
		
		uint8_t page1[8]; //9
		
		uint8_t page2[8]; //17
		uint8_t page3[8]; //25
		
		union{
			uint8_t page4[8];  //33
			struct {
				uint16_t tol_s8;
				uint16_t tol_d;
				uint16_t r_day_max;
				uint16_t r_week_max;
			};
		};
		union{
			uint8_t page5[8];  //41
			struct {
				uint8_t codeVOC; // immer 0x37 nach Neustart
				uint8_t days_of_r0; //Anzahl der Tage fuer die r0 ermittelt wird
				int8_t corr_VOC_mult; //r0 corr
				int8_t corr_VOC_div;
				uint16_t vv3;
				uint8_t time_corr; //Wiregate;
				uint8_t reset_code;
			};
			uint16_t page5d[4];
		};
		union{
			uint8_t page6[8]; //25
			struct {
				uint16_t R0;
				uint16_t VS;
				uint8_t cmode;
				int16_t ip;
				uint8_t free;
			};
		};
		uint8_t page7[8];  //57
		
	};
} pack2_t;
volatile pack2_t pack2;

//#define TIME_CORR 3
volatile int8_t time_corr_count;//=TIME_CORR;

#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
	#else
	ISR(WDT_vect) {
		#endif
		pack1.etm++;
		time_corr_count--;
		if ((pack1.page1[0]&0x7)==0) {
			 wdcounter++;
			} 
			if (time_corr_count<=0) {
				//wdcounter++;
				pack1.etm+=1;
				time_corr_count=pack2.time_corr;
			}
	/*	if ((pack1.page1[0]&0x0F)==0) {
			 pack1.etm+=2;

		}*/
/*		if ((pack1.page1[0]&0x1F)==0) {
			pack1.etm--;
		}*/

		if (reset_indicator==1) reset_indicator++;
		else if (reset_indicator==2) mode=0;


	}





volatile int16_t DS2438_1_TEMP;
volatile uint16_t DS2438_1_VAD;
volatile int16_t DS2438_2_TEMP;
volatile uint16_t DS2438_2_VAD;


uint8_t userRegister[1];
int16_t sRH,sT;
double temperatureC,humidityRH,hhum;
double l;

uint8_t max_adr=0;
#define CH0_M MAX1164x_C_SCAN0|MAX1164x_C_SGL
#define CH1_M MAX1164x_C_SCAN0|MAX1164x_C_SGL|MAX1164x_C_CS0
#define CH0_CH1 MAX1164x_C_SCAN0
//|MAX1164x_C_CS0

uint16_t weekmaxarr[33];

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
#define EEPROM_CODE_DAYOFR0 8
#define EEPROM_CORR 10
#define EEPROM_FREE 12
#define EEPROM_TCORR_RESET 14


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
	(1<<WDP2) | (1<<WDP1);    // Set Timeout to ~1 seconds

	MCUSR=0;

	pack1.temp=0x0550;
	pack2.page3[0]=0xF1;
	pack2.cmode=0;
	pack2.R0=readEEPROM(EEPROM_R0,1);
	R0=pack2.R0/100.0;
		
	pack2.r_day_max=readEEPROM(EEPROM_R0d,1);
	pack2.r_week_max=readEEPROM(EEPROM_R0w,1);
	pack2.tol_d=readEEPROM(EEPROM_dol,0); 
	pack2.tol_s8=0;  //Tag faengt mit Einschalten an
	

	pack2.page5d[0]=readEEPROM(EEPROM_CODE_DAYOFR0,0x0437);
	pack2.page5d[1]=readEEPROM(EEPROM_CORR,0x0101);
	pack2.page5d[2]=readEEPROM(EEPROM_FREE,0x0);
	pack2.page5d[3]=readEEPROM( EEPROM_TCORR_RESET,0x0005);
	time_corr_count=pack2.time_corr;

	for(uint8_t i=0;i<pack2.days_of_r0;i++) {
		weekmaxarr[i]=pack2.r_week_max;
	}
	if (testSW()) {
		config_info2[5]=12;
	}else{
		config_info2[5]=7;
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
			if ((cpsp2&0x80)!=0) {
				if ((cpsp2&0x0F)==5) {
					if (pack2.reset_code==0x01) {
						R0=1;
						pack2.R0=0;
						writeEEPROM(EEPROM_R0,0);
					} else if (pack2.reset_code==0x05) {
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

					} else {
						writeEEPROM(EEPROM_CODE_DAYOFR0,0x0037|(pack2.days_of_r0<<8));
						writeEEPROM(EEPROM_CORR,pack2.page5d[1]);
						writeEEPROM(EEPROM_FREE,pack2.page5d[2]);
						writeEEPROM(EEPROM_TCORR_RESET,pack2.time_corr);
					}
				}
				cpsp2=0;
		}
		if (wdcounter>0) {  //8s
			pack2.tol_s8++;
			
			//pack1.dis+=8;
			//pack1.eoc+=8;
			if (pack2.tol_s8>(10000))  {//10800 ist theortisch der Tag aber meistens zu lang
				pack2.tol_s8=0;
				pack2.tol_d++;  //rund 180 Jahre :-)
				pack2.r_week_max=0;
				weekmaxarr[pack2.days_of_r0]=pack2.r_day_max;
				for(uint8_t i=0;i<pack2.days_of_r0;i++) {
					weekmaxarr[i]=weekmaxarr[i+1];
					//maximum of week
					if (weekmaxarr[i]>pack2.r_week_max) pack2.r_week_max=weekmaxarr[i];
				}
				if (pack2.tol_d>pack2.days_of_r0) {
					pack2.R0=pack2.r_week_max;
				} else {
					pack2.R0=pack2.r_day_max;
				}
				//R0=//R0-0.5*(pack2.R0/100-R0);
				//R0=R0-(R0-pack2.R0/100.0)*0.5	;
				R0=pack2.R0/100.0;
				pack2.R0=R0*100;
				writeEEPROM(EEPROM_R0,pack2.R0);
				writeEEPROM(EEPROM_R0d,pack2.r_day_max);  //Maximum des Tages
				writeEEPROM(EEPROM_R0w,pack2.r_week_max); //Maximum der Letzten 7 Tage
				writeEEPROM(EEPROM_dol,pack2.tol_d); //Anzahl der Betriebstage
				pack2.r_day_max=0;
			}
			if (startup!=0) startup--;
			getSHT2xHumTemp(&temperatureC,&humidityRH);
			ip=interp(temperatureC,humidityRH);
			pack2.ip=ip*1000;
			double RH=calibr_hum(temperatureC,-0.2,humidityRH)*10.0;
			double TC =temperatureC *10.0-2;


			if (testSW()) {
				 DS2438_2_VAD=RH;
				 DS2438_2_TEMP=TC*25.6;
				 
				 config_info2[5]=12;	//10V = 100%
			}else{
				hhum=(1.0546-0.000216*TC)*(RH);
				//am2302_hum=0.318*hhum +76.0;
				DS2438_2_VAD=0.31*hhum +80;
				DS2438_2_TEMP=TC*25.6;
				config_info2[5]=7;
			}
			DS2438_1_TEMP=DS2438_2_TEMP;
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
			pack2.VS=mr*5/2;
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
			l=exp((1-(l/R0))*6.05)-1;// exp((1-($val)/55)*5.75);  (5.75 geht über 125 6.05 geht bis 240... mittlere Linie im Datenblatt)
			//l=(l/R0*100.0);
			l=l*0.5*35; //fuer DS2438
			l=l*(double)pack2.corr_VOC_mult/(double)pack2.corr_VOC_div;
			if (l==0) l=1;
			gcontrol=1;
			wdcounter=0;
			
		}
	
		if (gcontrol==1) {
			uint16_t w=l;
			DS2438_1_VAD=w;
			
			gcontrol=0;
		}
		if ((gcontrol==2)||(gcontrol==3)) {
			gcontrol=0;
			
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
		asm("SLEEP");
   }


}
