#ifndef MAX1164x_H
#define MAX1164x_H
#define MAX1164x_S_SEL2 0x40
#define MAX1164x_S_SEL1 0x20
#define MAX1164x_S_SEL0 0x10
#define MAX1164x_S_ECLK 0x08
#define MAX1164x_S_BIP 0x04
#define MAX1164_S_RESET 0x02

#define MAX1164x_C_SCAN1 0x40
#define MAX1164x_C_SCAN0 0x20
#define MAX1164x_C_CS0  0x02
#define MAX1164x_C_SGL 0x01



void MAX1164x_init(void);
void MAX1164x_config(uint8_t setup,uint8_t config);
int16_t MAX1164x_read(void);


#endif
