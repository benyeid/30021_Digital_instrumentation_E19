#ifndef __GYRO_H
#define __GYRO_H
#include "stm32f30x_conf.h"
#include "gpio.h"
#include <inttypes.h>

#define GYRO_CS_LOW() (GPIOD->ODR &=  ~(0x0001 << 2))
#define GYRO_CS_HIGH() (GPIOD->ODR |=  (0x0001 << 2))
#define GYRO_CTRL_REG1 0x20
#define GYRO_CTRL_REG2 0x21
#define GYRO_OUT_TEMP 0x26
#define GYRO_OUT_STAT 0x27
#define GYRO_OUT_X_L 0x28
#define GYRO_OUT_X_H 0x29
#define GYRO_OUT_Y_L 0x2A
#define GYRO_OUT_Y_H 0x2B
#define GYRO_OUT_Z_L 0x2C
#define GYRO_OUT_Z_H 0x2D
#define GYRO_WHOAMI 0x0F

void gyro_read(int16_t *x, int16_t *y, int16_t *z);
uint8_t spi3_recv_byte();


void setup_gyro();
uint8_t gyro_read_reg(uint8_t addr);
void gyro_write_reg(uint8_t addr, uint8_t data);
void gyro_reset();

uint8_t gyro_temp();
//!!!
void Gyro_temp();
uint8_t gyro_status();
uint8_t gyro_whoami();
void gyro_read(int16_t *x, int16_t *y, int16_t *z);
void init_spi_gyro();

void Gyro_test();
#endif
