#ifndef BMP280_H
#define BMP280_H



void bmp280Init(void);
int32_t bmp280ReadTemp(void);
int32_t bmp280ReadPressure(uint8_t oss);
int32_t bmp280ReadPressure_simple();
void bmp280Convert(double * temperature, double * pressure,uint8_t oss);
void bmp280ConvertInt(int32_t * temperature,uint32_t *pressure,uint8_t oss);
void bmp280ConvertInt_df(int32_t * temperature,uint32_t *pressure,uint8_t oss);
void bmp280ConvertIntP(int32_t temp256,uint32_t *pressure) ;
void bmp280ConvertIntP1(uint32_t *pressure) ;

#endif
