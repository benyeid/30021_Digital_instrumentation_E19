#include "registers.h"
#include <inttypes.h>

void setup_Acc();

uint8_t Acc_ReadRegister(uint8_t addr);
void Acc_WriteReg(uint8_t addr, uint8_t data);
void Acc_reset();
uint8_t Acc_status();
uint8_t Acc_whoami();
void Acc_read(int16_t *x, int16_t *y, int16_t *z);
void init_Acc();
void Acc_test();


//void getAccValues(uint16_t *x, uint16_t *y, uint16_t *z);

