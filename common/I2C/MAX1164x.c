#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "USI_TWI_Master.h"
#include "MAX1164x.h"

void MAX1164x_init(){
	
	
}
void MAX1164x_config(uint8_t setup,uint8_t config){
	I2c_StartCondition();
	I2c_WriteByte (0b01101100);
	I2c_WriteByte (setup|0x80);
	I2c_WriteByte (config);
	I2c_StopCondition();
	
}
int16_t MAX1164x_read(void) {
	uint16_t res;
	I2c_StartCondition();
	I2c_WriteByte (0b01101101);
	//DDR_USI&=~(1<<PIN_USI_SCL);
	//while ((PIN_USI&(1<<PIN_USI_SCL))==0);
	//DDR_USI|=(1<<PIN_USI_SCL);
	_delay_us(80);
	res=((int16_t)(I2c_ReadByte(ACK)&0x0F))<<8;
	res|=I2c_ReadByte(NO_ACK);
	
	
	I2c_StopCondition();
	return res;
	
}


int16_t MAX1164x_read8(void) {
	uint16_t res;
	I2c_StartCondition();
	I2c_WriteByte (0b01101101);
	//DDR_USI&=~(1<<PIN_USI_SCL);
	//while ((PIN_USI&(1<<PIN_USI_SCL))==0);
	//DDR_USI|=(1<<PIN_USI_SCL);
	_delay_us(80);
	res=((int16_t)(I2c_ReadByte(ACK)&0x0F))<<8;
	res|=I2c_ReadByte(NO_ACK);
	
	
	I2c_StopCondition();
	return res;
	
}
