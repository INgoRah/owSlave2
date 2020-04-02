
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

extern void OWINIT();
extern void EXTERN_SLEEP();

const float k_rs[54] PROGMEM ={0.000000,24.125000,48.585366,72.731707,96.829268,121.097561,145.700000,170.600000,195.650000,220.625000,245.365854,269.853659,294.119048,318.195122,342.166667,366.000000,389.761905,413.428571,437.023810,460.558140,484.047619,507.511628,530.976190,554.418605,577.883721,601.395349,624.952381,648.571429,672.285714,696.073171,719.976190,744.000000,768.146341,792.439024,816.853659,841.414634,866.125000,890.975000,916.000000,941.179487,966.525000,992.025641,1017.717949,1043.589744,1069.657895,1095.945946,1122.432432,1149.184211,1176.189189,1203.472222,1231.083333,1259.000000,1287.285714,1315.941176};
//const float j_rs[70] PROGMEM ={0, 18.302913, 34.830476, 50.783019, 70.653704, 90.505455, 110.341818, 130.165455, 149.163636, 160.791071, 180.596364, 200.398214, 220.200000, 240.000000, 250.882883, 270.603636, 290.409091, 310.216364, 330.025455, 342.472727, 360.649091, 380.461818, 400.275000, 420.087273, 435.275676, 450.703636, 470.503636, 490.298214, 510.082456, 523.486726, 540.621053, 560.370175, 580.105172, 591.979487, 610.527119, 630.213559, 644.601653, 660.534426, 680.168852, 690.787097, 710.391935, 729.123810, 740.559375, 760.126562, 770.684615, 790.235385, 800.782812, 820.331250, 834.681250, 850.446032, 870.017460, 880.600000, 900.196774, 911.099187, 930.432787, 950.073333, 960.728333, 980.396667, 1000.078333, 1010.772881, 1030.475862, 1050.187931, 1065.717241, 1080.631034, 1100.358621, 1120.089655, 1131.840000,1150.556897, 1170.294737, 1190.035088};
	double gettemp_rs(double V) {
		uint8_t iv=(uint8_t)(V);
		float t0=pgm_read_float(&(k_rs[iv]));
		float t1=pgm_read_float(&(k_rs[iv+1]));
		return t0+(t1-t0)/1*(V-iv);
	}


uint8_t owid[8]={0x28, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x02, 0xAC};/**/
uint8_t config_info[26]={0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x02,6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	
#if (owid>128) 
#error "Variable not correct"
#endif

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;


volatile uint8_t wdcounter;


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
} pack_t;
volatile pack_t pack;




#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
#else
ISR(WDT_vect) {
#endif 
	//sleep_disable();          // Disable Sleep on Wakeup
	wdcounter++;
	if (reset_indicator==1) reset_indicator++;
	else if (reset_indicator==2) mode=0;
/*	if (timeout==2) {
		DIS_TIMER;
		EN_OWINT;
		mode=OWM_SLEEP;
	}
	timeout++;*/
	//sleep_enable();           // Enable Sleep Mode

}


#define OWM_PORT PORTA
#define OWM_PIN PINA
#define OWM_PINN PINA0
#define OWM_DD DDRA

#define OWM_SET_LOW OWM_PORT&=~(1<<OWM_PINN);OWM_DD|=(1<<OWM_PINN)
#define OWM_SET_HIGH OWM_DD&=~(1<<OWM_PINN);OWM_PORT|=(1<<OWM_PINN)

#define OWM_IS_LOW ((OWM_PIN & (1<<OWM_PINN))==0)


void owm_init() {
	OWM_PORT|=(1<<OWM_PINN); //PULL UP
	OWM_DD&=~(1<<OWM_PINN);
}

#define owm_delay(us1)  _delay_us(us1)

uint8_t owm_reset() {
	OWM_SET_LOW;
	owm_delay(480);
	OWM_SET_HIGH;
	owm_delay(60);
	if (OWM_IS_LOW) {owm_delay(420); return 1;} else {owm_delay(420); return 0;}
	
	
}

void owm_rw(uint8_t *b) {
	uint8_t i;
	uint8_t pp=1;
	for(i=0;i<8;i++) {
		if (pp&b[0]) {
			OWM_SET_LOW;
			owm_delay(6);
			OWM_SET_HIGH;
			owm_delay(9);
			if (OWM_IS_LOW) {
				b[0]&=~pp;
			}
			owm_delay(80-6-9);
			
			} else {
			OWM_SET_LOW;
			owm_delay(60);
			OWM_SET_HIGH;
			owm_delay(20);
		}
		pp=(pp<<1);
	}
}

void owm_block(uint8_t count, uint8_t *buf){
	uint8_t i;
	for(i=0;i<count;i++) {
		owm_rw(buf+i);
	}
}

inline int16_t ow_fconvert(uint8_t b1, uint8_t b2) {
	int16_t tsht;
	tsht=b1  |((int)b2<<8);
	if (b2 & 0x080)
	tsht |= 0xFFFFF0000;
	return tsht;
}
volatile double V,ktemp;

uint16_t ADmess() {
	 ADMUX=0b10101101;
	 ADCSRA|=(1<<ADSC);
	 while ((ADCSRA&(1<<ADSC)));
	return ADC;
}

int main(void){
    //PRR|=(1<<PRUSI)|(1<<PRADC);  //Switch off usi and adc for save Power
	pack.temp=0x0550;
	pack.config=0x7F;
	pack.TH=75;
	pack.TL=70;
	pack.rrFF=0xFF;
	pack.rr00=0;
	pack.rr10=0x10;
	PORTA=0xFF-(1<<PINA1)-(1<<PINA2);
	PORTB=0xFF;
	OWINIT();

	MCUCR &=~(1<<PUD); //All Pins Pullup...
	MCUCR |=(1<<BODS);

	WDTCSR |= ((1<<WDCE) );   // Enable the WD Change Bit//| (1<<WDE)
	WDTCSR |=   (1<<WDIE) |              // Enable WDT Interrupt
	(1<<WDP2) | (1<<WDP1);   // Set Timeout to ~1 seconds
	MCUSR=0;
	uint8_t block[13];
	sei();
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	owm_init();
	owm_reset();
	block[0]=0xCC;
	block[1]=0x4E;
	block[2]=0;
	block[3]=0;
	block[4]=0x1F;
	owm_block(5,block);
	owm_reset();
	block[0]=0x33;
	for(uint8_t i=1;i<9;i++) {
		block[i]=0xFF;
	}
	owm_block(9,block);
	
	if (block[1]==0x28) {  //DS18B20 angeschlossen
		for(uint8_t i=0;i<8;i++) {
			owid[i]=block[i+1];
		}
		while(EECR & (1<<EEPE));
		EEAR=E2END-7;
		EECR|=(1<EERE);
		if (EEDR!=0x28) { //Wenn keine ID im Eeprom uebernimm es
			for(uint8_t a=0;a<8;a++) {
				while(EECR & (1<<EEPE));
				EECR = (0<<EEPM1)|(0<<EEPM0);
				EEAR = E2END-7+a;
				EEDR = block[a+1];
				EECR |= (1<<EEMPE);
				EECR |= (1<<EEPE);
			}
		}
	}
	
	uint16_t ares[16],sum;
	uint8_t par=0;
	ares[0]=0;//ADmess();
	for (par=1;par<16;par++) {
		ares[par]=ares[0];
	}
	par=0;
	wdcounter=0;
	gcontrol=1;

    while(1)   {
		if (wdcounter>0) {
			ares[par]=ADmess();
			par++;
			if (par>15) par=0;
			wdcounter=0;
		}
		if (gcontrol) {
			PORTB|=(1<<PORTB0);
			sum=0;
			for(uint8_t i=0;i<16;i++) {
				sum+=ares[i];
			}
			V=sum/20.0/1024.0*1.12*1000.0/16.0;
			//V=sum/20.0/1024.0*1.01*1000.0/16.0;
			ktemp=gettemp_rs(V);
			owm_reset();
			block[0]=0xCC;
			block[1]=0x44;
			owm_block(2,block);
			_delay_ms(100);
			owm_reset();
			block[0]=0xCC;
			block[1]=0xBE;
			for(uint8_t i=0;i<9;i++) block[i+2]=0xFF;
			owm_block(11,block);
			uint16_t htemp;
			if (PINB&(1<<PINB0)) {
				htemp=(ktemp*16+(block[2]|(block[3]<<8)))/10;
			} else {
				
				htemp=ktemp*16+(block[2]|(block[3]<<8));
			}
			uint8_t t8=pack.temp>>4;
			uint8_t af=0;
			if (t8>pack.TH) af=1;
			if (t8<=pack.TL) af=1;
			cli();
			pack.temp=htemp;
			alarmflag=af;
			sei();
			EXTERN_SLEEP();
			PORTB&=~(1<<PORTB0);
		}

		
#if  defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)  || defined(__AVR_ATtiny85__)
			if (((TIMSK & (1<<TOIE0))==0)&& (mode==0))
#endif			
#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) ||defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
			if (((TIMSK0 & (1<<TOIE0))==0)&& (mode==0))
#endif
			  {
//			CLKPR=(1<<CLKPCE);
	//		CLKPR=(1<<CLKPS2); /*0.5Mhz*/
//			PORTB&=~(1<<PINB1);
			MCUCR|=(1<<SE)|(1<<SM1);
			MCUCR&=~(1<<ISC01);
		} else {
			MCUCR|=(1<<SE);
			MCUCR&=~(1<<SM1);
		}
		asm("SLEEP");
   }


}