#ifndef REGISTERS_H_INCLUDED
#define REGISTERS_H_INCLUDED

#include "stm32f30x_conf.h"
#include "gpio.h"

///GYRO REGISTERS

#define GYRO_CS_GPIO GPIOD
#define GYRO_CS_PIN 2

#define GYRO_CS_LOW() (GYRO_CS_GPIO->ODR &=  ~(0x0001 << GYRO_CS_PIN))
#define GYRO_CS_HIGH() (GYRO_CS_GPIO->ODR |=  (0x0001 << GYRO_CS_PIN))

#define WHO_AM_I 0x0F

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
/*
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define REFERENCE 0x25 */

#define OUT_TEMP 0x26
#define GYRO_OUT_STAT 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38


///MAGNETOMETER REGISTERS

#define MAG_CS_GPIO GPIOB
#define MAG_CS_PIN 0

#define MAG_CS_LOW() (MAG_CS_GPIO->ODR &=  ~(0x0001 << MAG_CS_PIN))
#define MAG_CS_HIGH() (MAG_CS_GPIO->ODR |=  (0x0001 << MAG_CS_PIN))

#define WHO_AM_I_M 0x0F
#define ACT_THS_M 0x1e
#define ACT_DUR_M 0x1f
#define CTRL_REG1_M 0x20
#define CTRL_REG2_M 0x21
#define CTRL_REG3_M 0x22
#define CTRL_REG4_M 0x23
#define CTRL_REG5_M 0x24

#define CTRL_REG6_M 0x25
#define CTRL_REG7_M 0x26

#define MAG_OUT_STAT 0x27
#define OUT_X_L_M 0x28
#define OUT_X_H_M 0x29
#define OUT_Y_L_M 0x2A
#define OUT_Y_H_M 0x2B
#define OUT_Z_L_M 0x2C
#define OUT_Z_H_M 0x2D

#define TEMP_L_M 0x2E
#define TEMP_H_M 0x2F

#define INT_CFG_M 0x30
#define INT_SRC_M 0x31

#define INT_TSH_L_M 0x32
#define INT_TSH_H_M 0x33

///ACCELEROMETER REGISTERS

#define ACCEL_CS_GPIO GPIOA
#define ACCEL_CS_PIN 15

#define ACCEL_CS_LOW() (ACCEL_CS_GPIO->ODR &=  ~(0x0001 << ACCEL_CS_PIN))
#define ACCEL_CS_HIGH() (ACCEL_CS_GPIO->ODR |=  (0x0001 << ACCEL_CS_PIN))

#define WHO_AM_I_A 0x0F
#define ACT_THS_A 0x1e
#define ACT_DUR_A 0x1f
#define CTRL_REG1_A 0x20
#define CTRL_REG1_A_CONTENT 0b11100111 //see documentation point 8.4@page33 and table 25
//High resolution enabled, ODR 800HZ, continous update, all axis enabled
#define CTRL_REG2_A 0x21
#define CTRL_REG2_A_CONTENT 0b00010111 // see point 8.5 and table 26
//high-pass normal normal mode, data from internal reg sent to output, HP filter for IT generatior enabled
#define CTRL_REG3_A 0x22
#define CTRL_REG3_A_CONTENT 0b11111000 // see point 8.6
//all interrupts enabled
#define CTRL_REG4_A 0x23
#define CTRL_REG4_A_CONTENT 0b00000111 // see point 8.7
//anti-aliasing 400Hz, full scale +-2g,bandwidth automatically sel., reg address automatically incremented, i2c enabled, SIM read and write
#define CTRL_REG5_A 0x24
#define CTRL_REG5_A_CONTENT 0b10010100
// see point 8.8 and table 35. Debug enabled, update/2samplespositive sign sef-test, IT active H, push-pull selection
#define CTRL_REG6_A 0x25
#define CTRL_REG6_A_CONTENT 0b0000000 //see point 8.9, only MSB is significant
#define CTRL_REG7_A 0x26
#define CTRL_REG7_A_CONTENT 0b00111100 //see point 8.10.
//Duration counter decremented, latched IT enabled

#define ACC_STATUS_REG 0x27
#define STATUS_REG_CONTENT 0b00000000 // see 8.11, no overrun data available

#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D

#define FIFO_CTRL 0x2E // see table 50
#define FIFO_SRC 0x2F

#define IG_CFG1_A 0x30
#define IG_SRC1_A 0x31

#define IG_TSH_X1_A 0x32
#define IG_TSH_Y1_A 0x33
#define IG_TSH_Z1_A 0x34

#define IG_DUR1_A 0x35
#define IG_CFG2_A 0x36
#define IG_SRC2_A 0x37
#define IG_THS2_A 0x38
#define IG_DUR2_A 0x39

#define XL_REFERENCE 0x3a
#define XH_REFERENCE 0x3b
#define YL_REFERENCE 0x3C
#define YH_REFERENCE 0x3d
#define ZL_REFERENCE 0x3e
#define ZH_REFERENCE 0x3f



#endif /* REGISTERS_H_INCLUDED */
