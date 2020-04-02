// Copyright (c) 2015, Tobias Mueller tm(at)tm3d.de
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
#define FP_CALC
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

extern void OWINIT();


uint8_t owid[8]={0x26, 0xA2, 0xD9, 0x84, 0xDD, 0xDD, 0x05, 0xCE};/**/
uint8_t config_info[16]={0x01,0x06, 0x05,0x08, 0x04,0x07, 0x00,0x00, 0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	

extern uint8_t mode;
extern uint8_t gcontrol;
extern uint8_t reset_indicator;
extern uint8_t alarmflag;

volatile uint8_t wdcounter;


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
} pack_t;
volatile pack_t pack;

#if  defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)  || defined(__AVR_ATtiny85__)

#define DDR_SENSOR   DDRB 
#define PORT_SENSOR  PORTB
#define PIN_SENSOR   PINB
#define SENSOR       PB4
#endif

#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
#define DDR_SENSOR   DDRA
#define PORT_SENSOR  PORTA
#define PIN_SENSOR   PINA
#define SENSOR       PINA2
#endif



#define SENSOR_sda_out		DDR_SENSOR |= (1 << SENSOR)
#define SENSOR_sda_in			DDR_SENSOR &= ~(1 << SENSOR);PORT_SENSOR |= (1 << SENSOR) // release sda => hi in consequence of pullup
#define SENSOR_sda_low    PORT_SENSOR &= ~(1 << SENSOR)
#define SENSOR_is_hi			PIN_SENSOR & (1 << SENSOR)
#define SENSOR_is_low		!(PIN_SENSOR & (1 << SENSOR))

volatile int16_t am2302_temp;
volatile uint16_t am2302_hum;


uint8_t am_wait(uint8_t _time,uint8_t _signal){
	TCNT1=0;
	while(TCNT1==0);
	if (_signal) 
		while((SENSOR_is_hi)&&(TCNT1<_time)) {}
	else
		while((SENSOR_is_low)&&(TCNT1<_time)) {}
	 
		if (TCNT1>=_time) {return 1;}
	return 0;
}

volatile uint8_t am2302_mode=0;	
volatile uint8_t timeout=0;



#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
ISR(WATCHDOG_vect) {
#else
ISR(WDT_vect) {
#endif 
	sleep_disable();          // Disable Sleep on Wakeup
	am2302_mode++;
	if (reset_indicator==1) reset_indicator++;
	else if (reset_indicator==2) mode=0;
/*	if (timeout==2) {
		DIS_TIMER;
		EN_OWINT;
		mode=OWM_SLEEP;
	}
	timeout++;*/
	sleep_enable();           // Enable Sleep Mode

}

int testSW() {
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
	return (r==0);
	
	
}	
	
uint8_t am2302_1() {
	int16_t lam2302_temp;
	uint16_t lam2302_hum;
	uint8_t rSREG;
	uint8_t sensor_data[5];
#if  defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)  || defined(__AVR_ATtiny85__)
	TCCR1=(1<<CS12); //Clock/8 1탎
#endif

#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
	TCCR1B=(1<<CS11); //Clock/8 1탎
#endif
	SENSOR_sda_out;
	SENSOR_sda_low;	// MCU start signal
	TCNT1=0;while(TCNT1==0);while(TCNT1<250) {}// start signal (pull sda down for min 0.8ms and maximum 20ms)
	TCNT1=0;while(TCNT1==0);while(TCNT1<250) {}// start signal (pull sda down for min 0.8ms and maximum 20ms)
	SENSOR_sda_in;
	if (am_wait(200,1)) return 2;

	// AM2302 response signal min: 75us typ:80us max:85us
	if (am_wait(100,0)) return 3;
	if (am_wait(100,1)) return 4;
	
	for(uint8_t i = 0; i < 5; i++)	{
		uint8_t sensor_byte = 0;
		for(uint8_t j = 1; j <= 8; j++) {// get 8 bits from sensor
			if (am_wait(88,0)) return 5;
			
			TCNT1=0;while(TCNT1==0);;while(TCNT1<35)		;
			sensor_byte <<= 1; // add new lower byte
			if (SENSOR_is_hi) {// if sda high after 30us => bit=1 else bit=0
				sensor_byte |= 1;
				if (am_wait(45,1)) return 6;// 30us - 75us = 45us
			}
		}
		sensor_data[i] = sensor_byte;
	}

	// checksum
	if ( ((sensor_data[0]+sensor_data[1]+sensor_data[2]+sensor_data[3]) & 0xff ) != sensor_data[4])
	{
		// debug output
		//printf("%b %b %b %b %b %b" CR, sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3], sensor_data[4], ((sensor_data[0]+sensor_data[1]+sensor_data[2]+sensor_data[3]) & 0xff ));
		PORTB&=~(1<<PINB0);
		return 7;
	}
	if (!testSW()) {
#ifdef FP_CALC
	double htemp;
	if (sensor_data[2]&0x80) { //min
		sensor_data[2]&=~(0x80);
		htemp=-((sensor_data[2]<<8) + sensor_data[3]);
	} else
		htemp=((sensor_data[2]<<8) + sensor_data[3]);
	double hhum=(1.0546-0.000216*htemp)*((sensor_data[0]<<8) + sensor_data[1]);
	
	lam2302_hum=0.318*hhum +76;
	lam2302_temp=htemp*25.6;
#else

	if (sensor_data[2]&0x80) { //minus
		sensor_data[2]&=~(0x80);
		lam2302_temp=-((sensor_data[2]<<8) + sensor_data[3]);
	} else
		lam2302_temp=((sensor_data[2]<<8) + sensor_data[3]);
	
	lam2302_hum=((sensor_data[0]<<8) + sensor_data[1]);
	volatile uint32_t h1=lam2302_temp*lam2302_hum*3/44803;
	lam2302_hum=lam2302_hum*16/49-h1+80;
	//lam2302_temp=lam2302_temp*128/5;

	int16_t h2=lam2302_temp%5;
	lam2302_temp=lam2302_temp/5;
	lam2302_temp*=128;
	lam2302_temp+=h2*128/5;
#endif    
	config_info[5]=7;
	}
	else {
		if (sensor_data[2]&0x80) { //minus
			sensor_data[2]&=~(0x80);
			lam2302_temp=-((sensor_data[2]<<8) + sensor_data[3]);
		} else
		lam2302_temp=((sensor_data[2]<<8) + sensor_data[3])*25.6;;
		
		lam2302_hum=((sensor_data[0]<<8) + sensor_data[1]);
		config_info[5]=12;
	}
	rSREG=SREG;
	cli();
	am2302_hum=lam2302_hum;
	am2302_temp=lam2302_temp;
	SREG=rSREG;
	return 0;
}


uint8_t am2302_2() { //4mhz
	int16_t lam2302_temp;
	uint16_t lam2302_hum;
	uint8_t rSREG;
	uint8_t sensor_data[5];
	#if  defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)  || defined(__AVR_ATtiny85__)
	TCCR1=(1<<CS11)|(1<<CS10); //Clock/8 1탎
	#endif

	#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
	TCCR1B=(1<<CS11); //Clock/8 1탎
	#endif
	SENSOR_sda_out;
	SENSOR_sda_low;	// MCU start signal
	TCNT1=0;while(TCNT1==0);while(TCNT1<250) {}// start signal (pull sda down for min 0.8ms and maximum 20ms)
	TCNT1=0;while(TCNT1==0);while(TCNT1<250) {}// start signal (pull sda down for min 0.8ms and maximum 20ms)
	SENSOR_sda_in;
	if (am_wait(200,1)) return 2;

	// AM2302 response signal min: 75us typ:80us max:85us
	if (am_wait(100,0)) return 3;
	if (am_wait(100,1)) return 4;
	
	for(uint8_t i = 0; i < 5; i++)	{
		uint8_t sensor_byte = 0;
		for(uint8_t j = 1; j <= 8; j++) {// get 8 bits from sensor
			if (am_wait(88,0)) return 5;
			
			TCNT1=0;while(TCNT1==0);;while(TCNT1<35)		;
			sensor_byte <<= 1; // add new lower byte
			if (SENSOR_is_hi) {// if sda high after 30us => bit=1 else bit=0
				sensor_byte |= 1;
				if (am_wait(45,1)) return 6;// 30us - 75us = 45us
			}
		}
		sensor_data[i] = sensor_byte;
	}

	// checksum
	if ( ((sensor_data[0]+sensor_data[1]+sensor_data[2]+sensor_data[3]) & 0xff ) != sensor_data[4])
	{
		// debug output
		//printf("%b %b %b %b %b %b" CR, sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3], sensor_data[4], ((sensor_data[0]+sensor_data[1]+sensor_data[2]+sensor_data[3]) & 0xff ));
		PORTB&=~(1<<PINB0);
		return 7;
	}
#ifdef FP_CALC
	lam2302_hum=0.318* ((sensor_data[0]<<8) + sensor_data[1])+76;
	if (sensor_data[2]&0x80) { //minus
		sensor_data[2]&=~(0x80);
		lam2302_temp=-((sensor_data[2]<<8) + sensor_data[3])*25.6;
	} else
		lam2302_temp=((sensor_data[2]<<8) + sensor_data[3])*25.6;
#else


	if (sensor_data[2]&0x80) { //minus
		sensor_data[2]&=~(0x80);
		lam2302_temp=-((sensor_data[2]<<8) + sensor_data[3]);
	} else
	lam2302_temp=((sensor_data[2]<<8) + sensor_data[3]);
	
	lam2302_hum=((sensor_data[0]<<8) + sensor_data[1]);
	volatile uint32_t h1=lam2302_temp*lam2302_hum*3/44803;
	lam2302_hum=lam2302_hum*16/49-h1+80;
	//lam2302_temp=lam2302_temp*128/5;

	int16_t h2=lam2302_temp%5;
	lam2302_temp=lam2302_temp/5;
	lam2302_temp*=128;
	lam2302_temp+=h2*128/5;
#endif	
	
	
	rSREG=SREG;
	cli();
	am2302_hum=lam2302_hum;
	am2302_temp=lam2302_temp;
	SREG=rSREG;
	return 0;
}



int main(void){
    PRR|=(1<<PRUSI)|(1<<PRADC);  //Switch off usi and adc for save Power
#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
	PORTA=0xFF;
	PORTB=0xFF-(1<<PORTB0); //Schalter kann gegen Masse sein und zieht dann immer Strom
	DDRB|=(1<<PORTB0); //Als Ausgang und 0
#endif
	OWINIT();

	ACSR|=(1<<ACD);  //Disable Comparator
	ADCSRB|=(1<<ACME); //Disable Analog multiplexer
	MCUCR &=~(1<<PUD); //All Pins Pullup...
	MCUCR |=(1<<BODS);
	MCUCR &=~(1<<PUD);
	
#if  defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)  || defined(__AVR_ATtiny85__)

	PORTB|=(1<<PINB0)|(1<<PINB1)|(1<<PINB3)|(1<<PINB4)|(1<<PINB5);
	DDRB|=(1<<PINB1); //DBLINE
#define SENSON PORTB|=(1<<PINB0);
#define SENSOFF PORTB&=~(1<<PINB0);
	DDRB|=(1<<PINB0); //stromversorgung
	
	// Set up Watch Dog Timer for Inactivity
	WDTCR |= ((1<<WDCE) | (1<<WDE));   // Enable the WD Change Bit
	WDTCR =   (1<<WDIE) |              // Enable WDT Interrupt
	(1<<WDP2) | (1<<WDP1);   // Set Timeout to ~2 seconds
#endif

#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)


	DDRA|=(1<<PINA1); //stromversorgung
#define SENSON PORTA|=(1<<PINA1);
#define SENSOFF PORTA&=~(1<<PINA1);

	// Set up Watch Dog Timer for Inactivity
	WDTCSR |= (1<<WDCE) ;   // Enable the WD Change Bit
	WDTCSR =   (1<<WDIE) |              // Enable WDT Interrupt
	(1<<WDP2) | (1<<WDP1);   // Set Timeout to ~2 seconds
#endif

	
    uint8_t i;
	uint8_t err;
#if  defined(__AVR_ATtiny25__)
    for(i=0;i<16;i++) pack.bytes[i]=0;
#else
	uint8_t pn=1;
    for(i=0;i<64;i++) pack.bytes[i]=0;
#endif
	//pack.bytes[0]=1;
	//pack.bytes[1]=2;
	SENSON
	_delay_ms(4000);
	err =am2302_1();
	sei();
    while(1)   {
		alarmflag=1;
		if (am2302_mode==2) {
			err=am2302_1();
			if (err!=0) {
#if  defined(__AVR_ATtiny25__)
#else
				pack.page3[pn]=err;
				pn=pn+1;if (pn>31) pn=1;
#endif
			}
			SENSOFF
			am2302_mode=3;
		}
		if (am2302_mode>=8) {
			am2302_mode=0;
			SENSON
		}
#if  defined(__AVR_ATtiny25__)||defined(__AVR_ATtiny45__)  || defined(__AVR_ATtiny85__)
			if (((TIMSK & (1<<TOIE0))==0)&& (mode==0))
#endif			
#if  defined(__AVR_ATtiny24__)||defined(__AVR_ATtiny44__)  || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny24A__)||defined(__AVR_ATtiny44A__)  || defined(__AVR_ATtiny84A__)
			if (((TIMSK0 & (1<<TOIE0))==0)&& (mode==0))
#endif
			  {
//			CLKPR=(1<<CLKPCE);
	//		CLKPR=(1<<CLKPS2); /*0.5Mhz*/
			//PORTB&=~(1<<PINB1);
			MCUCR|=(1<<SE)|(1<<SM1);
			MCUCR&=~(1<<ISC01);
		} else {
			MCUCR|=(1<<SE);
			MCUCR&=~(1<<SM1);
		}
		asm("SLEEP");
   }


}