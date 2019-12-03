#ifndef GYRO_H_INCLUDED
#define GYRO_H_INCLUDED

#ifndef __GYRO_H
#define __GYRO_H
#include "gpio.h"
#include "stm32f30x_conf.h"
#define GYRO_CS_GPIO GPIOD
#define GYRO_CS_PIN 4

#define GYRO_CS_LOW() (GPIOA->ODR &=  ~(0x0001 << 4))
#define GYRO_CS_HIGH() (GPIOA->ODR |=  (0x0001 << 4))
#define GYRO_CTRL_REG1 0x20
#define GYRO_CTRL_REG2 0x21
#define GYRO_CTRL_REG3 0x22
#define GYRO_CTRL_REG4 0x23
#define GYRO_CTRL_REG5 0x24
#define GYRO_OUT_TEMP 0x26
#define GYRO_OUT_STAT 0x27
#define GYRO_OUT_X_L 0x28
#define GYRO_OUT_X_H 0x29
#define GYRO_OUT_Y_L 0x2A
#define GYRO_OUT_Y_H 0x2B
#define GYRO_OUT_Z_L 0x2C
#define GYRO_OUT_Z_H 0x2D
#define GYRO_WHOAMI 0x0F

void gyro_getValues();
void init_spi3();
void gyro_temp();
void gyro_status();
void gyro_test();
void gyro_send(uint8_t address, uint8_t data);
uint8_t gyro_read(uint8_t address);
#endif

#endif /* GYRO_H_INCLUDED */
