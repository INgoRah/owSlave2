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
//#define __4MHZ__
#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include "../common/I2C/TWI_Master.h"
#include "../common/I2C/HDC2010.h"
#include "../common/I2C/MAX44009.h"
#include "../common/owSlave_tools.h"

OWST_EXTERN_VARS



volatile uint8_t owid1[8]={0x26, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x05, 0x50};/**/
volatile uint8_t owid2[8]={0x26, 0xA3, 0xD9, 0x84, 0x00, 0x16, 0x05, 0x67};/**/
                                        //18
volatile uint8_t config_info1[26]={3,28, 0x05,0x08, 8,27, 0x00,0x00, 0x02,17,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
volatile uint8_t config_info2[26]={0x01,0x06, 0x05,0x08, 0x04,0x07, 11,0x08, 0x02,0x07,0x00,0x07,17,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

#if (owid>128)
#error "Variable not correct"
#endif

extern uint8_t cpsp2;

OWST_WDT_ISR

/*


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
*/


	typedef union {
		#if  defined(__AVR_ATtiny25__)
		uint8_t bytes[16];
		#else
		uint8_t bytes[64];
		#endif
		struct {
			uint8_t status;  //1
			int16_t temp;  //2
			uint16_t voltage;  //4
			int16_t current;  //6
			uint8_t threshold; //8
			
			uint8_t page1[8]; //9
			#if  defined(__AVR_ATtiny25__)
			#else
			uint8_t page2[8]; //17
			union{
				uint8_t page3[8]; //25
				struct {
					uint8_t free;
					uint16_t R0;
					uint16_t VS;
					uint8_t cmode;
					int16_t ip;
					
				};
			};
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
			uint8_t page6[8];  //49
			uint8_t page7[8];  //57
			
			#endif
		};
	} pack2_t;
	volatile pack2_t pack2,pack1;




	volatile int16_t DS2438_2_TEMP;
	volatile uint16_t DS2438_2_VAD;
	volatile uint16_t DS2438_2_VDD=0x01F4;
	volatile int16_t DS2438_1_TEMP;
	volatile uint16_t DS2438_1_VAD;
	volatile uint16_t DS2438_1_VDD=0x01F4;

	OWST_TESTSW


	double temperatureC,humidityRH,hhum;
	double l;


	uint16_t weekmaxarr[33];

	//Kompensieren der Abhänigkeit von RS/RO von Temperatur und Luftfeuchte
	inline double interp(double t, double h) {
		double h2;
		double t2;
		h2=h*h;
		t2=t*t;
		return 4.76111e-9*h2*t2-3.96956e-7*h2*t+0.0000408889*h2-1.07132e-6*h*t2+0.000115968*h*t-0.0101333*h+0.000163806*t2-0.0241179*t+1.80591;
	}
	inline double calibr_hum05(double t,double hum) {
		double y=-0.0006*t*t-0.2455*t-28.5902;
		return -(hum/y)+hum;
	}
	inline double calibr_hum075(double t,double hum) {
		double y=-0.0004*t*t-0.1636*t-18.9173;
		return -(hum/y)+hum;
	}
	inline double calibr_hum1(double t,double hum) {
		double y=-0.0003*t*t-0.1228*t-14.0808;
		return -(hum/y)+hum;
	}

	double R0;
	uint16_t mr;
	uint8_t startup=10;
	double ip;
	double la[4];
	uint8_t lainit=1;


	
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
		PORTA=0xFF-(1<<PINA1);
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

		//ADMUX=0b00001110; //ADC1 +   ADC3 -
		ADMUX=0x01 ; //PA3 single and  3V
		OWST_INIT_ADC
		//ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0);//|
		
		

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

		for(uint8_t i=0;i<pack2.days_of_r0;i++) {
			weekmaxarr[i]=pack2.r_week_max;
		}

		for(uint8_t i=0;i<8;i++) pack2.page6[i]=owid1[i];
		if (testSW()) {
			config_info2[5]=12;
			}else{
			config_info2[5]=7;
		}

		TWI_Master_Initialise();
		
		//initSHT2x();
		HDC2010_Init();
		
		_delay_ms(100);
		gcontrol=1;
		sei();
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
					if (pack2.tol_d>7) {
						pack2.R0=pack2.r_week_max;
					} else {
						pack2.R0=pack2.r_day_max;
					}
					//R0=//R0-0.5*(pack2.R0/100-R0);
					R0=R0-(R0-pack2.R0/100.0)*0.5	;
					pack2.R0=R0*100;
					writeEEPROM(EEPROM_R0,pack2.R0);
					writeEEPROM(EEPROM_R0d,pack2.r_day_max);  //Maximum des Tages
					writeEEPROM(EEPROM_R0w,pack2.r_week_max); //Maximum der Letzten 7 Tage
					writeEEPROM(EEPROM_dol,pack2.tol_d); //Anzahl der Betriebstage
					pack2.r_day_max=0;
				}
				if (startup!=0) startup--;
				//getSHT2xHumTemp(&temperatureC,&humidityRH);
				 HDC2010_Readf(&temperatureC,&humidityRH);
				ip=interp(temperatureC,humidityRH);
				pack2.ip=ip*1000;
				//double RH=calibr_hum(temperatureC,-0.2,humidityRH)*10.0;
				//double TC =temperatureC *10.0-2;
				double RH=calibr_hum075(temperatureC,humidityRH)*10.0;
				//double RH=humidityRH*10.0;
				double TC=temperatureC*10.0-7.5;


				if (testSW()) {
					DS2438_2_VAD=RH;
					DS2438_2_TEMP=TC*25.6;
					config_info2[5]=12;	//10V = 100%
				}else{
					hhum=(1.0546-0.000216*TC)*(RH);
					DS2438_2_VAD=0.31*hhum +80;
					DS2438_2_TEMP=TC*25.6;
					config_info2[5]=7;
				}
				mr=0;
				//Kritische Sektion !___Ein Breakpoint in dieser Section kann den TGS8100 zerstoeren!___________________________
				PORTB&=~(1<<PINB1); //Auf 0 Ziehen
				_delay_us(300);
				for(uint8_t i=0;i<32;i++) {
					ADCSRA|=(1<<ADSC);
					while ((ADCSRA&(1<<ADSC)));
					mr+=ADC;
				 }
				PORTB|=(1<<PINB1);
				//ENDE Kritische Sektion !______________________________

				//pack2.VS=mr*3/128;  //VS in Volt = VS/256  --> Nur zum Debug .... Speicher reicht sonst nicht....
				l=mr*1.8/32768.0;  //Spannung in v
				l=( 1.8/l- 1) *30; //l is resistance
				if (lainit) {
					la[0]=la[1]=la[2]=la[3]=l;  //smaller code
					lainit=0;
				} else {
					//for (uint8_t i=0;i<3;i++) {
					//	la[i]=la[i+1];
					//}
					la[0]=la[1];
					la[1]=la[2];
					la[2]=la[3];
					la[3]=l;
				}
				double lasum=la[0]+la[1]+la[2]+la[3];
				l=lasum/4.0;
				
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
				//l=exp((1-(l/R0))*6.05)-1;// exp((1-($val)/55)*5.75);  (5.75 geht über 125 6.05 geht bis 240... mittlere Linie im Datenblatt)
				//l=pow(R0/l,1.8)*3-3; 
				if (l!=0) {
					l=R0/l;
					//l=l*l*12-12;
					l=exp((1-(1/l))*6.05)-1;
					l=l*(double)pack2.corr_VOC_mult/(double)pack2.corr_VOC_div;
					l=l*8.0; //fuer DS18B20*/
				}	
				wdcounter=0;
				DS2438_1_VAD=l;
				DS2438_1_VDD=(l-DS2438_1_VAD)*1000.0;
				
				l=MAX44009getlux(1);
				if (l<0.030) l=0.030; //Darf nicht 0 sein. minimum -35°C Sensor minimum 0.045
				//double l=1000;
				l=log(l)*10*256;
				DS2438_1_TEMP=l;

				
			}
			
			
			if (gcontrol) {
				gcontrol=0;
				
			}

			
			OWST_MAIN_END
		}


	}
