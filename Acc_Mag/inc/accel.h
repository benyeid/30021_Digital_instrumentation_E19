#ifndef __ACCEL_H
#define __ACCEL_H
#include "stm32f30x_conf.h"
#include "gpio.h"

#define ACCEL_CTRL_REG1 0x20
#define ACCEL_CTRL_REG2 0x21
#define ACCEL_CTRL_REG3 0x22
#define ACCEL_CTRL_REG4 0x23
#define ACCEL_OUT_STAT 0x27
#define ACCEL_OUT_X_L 0x28
#define ACCEL_OUT_X_H 0x29
#define ACCEL_OUT_Y_L 0x2A
#define ACCEL_OUT_Y_H 0x2B
#define ACCEL_OUT_Z_L 0x2C
#define ACCEL_OUT_Z_H 0x2D
#define ACCEL_WHOAMI 0x0F

void accel_read(int16_t *x, int16_t *y, int16_t *z);
void init_accel();
//uint8_t gyro_temp();
void accel_status();
void accel_whoami();
void accel_getValues();
#endif
