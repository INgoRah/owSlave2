#ifndef TSL256x_H
#define TSL256x_H

#define TSL_GAIN16 0x10
#define TSL_GAIN1 0x00
#define TSL_INT_TIME1 0x00
#define TSL_INT_TIME2 0x01
#define TSL_INT_TIME3 0x02

void TSL256x_init();
void TSL256x_setup(uint8_t conf);
uint16_t TSL256x_Ch(uint8_t ch);
 uint32_t CalculateLux(unsigned int iGain, unsigned int tInt, unsigned int ch0, unsigned int ch1);

uint16_t TSL256x_Lux();

#endif
