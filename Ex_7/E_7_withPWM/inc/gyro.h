#ifndef __GYRO_H
#define __GYRO_H
#include "gpio.h"
#include "registers.h"

void gyro_read(int16_t *x, int16_t *y, int16_t *z);
void init_spi_gyro();
uint8_t gyro_temp();
uint8_t gyro_status();
uint8_t gyro_whoami();
#endif
