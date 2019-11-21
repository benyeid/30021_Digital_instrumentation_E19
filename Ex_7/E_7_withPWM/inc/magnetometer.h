#include "registers.h"
#include <inttypes.h>

uint8_t MagReadRegisters(uint8_t addr);
void MagWriteRegisters(uint8_t addr, uint8_t data);
void Mag_Reset();
uint8_t Mag_Status();
uint8_t Mag_Whoami();
void mag_read(int16_t *x, int16_t *y, int16_t *z);
void init_mag();

/*
void setup_Mag();
void Mag_test();
void getMagValues(uint16_t *x, uint16_t *y, uint16_t *z);
*/