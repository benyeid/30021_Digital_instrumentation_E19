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

/// \brief Read gyro data from given address
/// \param uint8_t address
/// Gyro is connected to SPI3, GPIOA, it is hard-coded in the function!
/// Operation:
/// - write GPIOA ODR to capture port
/// - Wait until SPI_I2S_FLAG_TXE is set
/// - Send address on SPI3
/// - Wait until SPI_I2S_FLAG_RXNE is set
/// - Read byte = 0xFF & SPI_I2S_ReceiveData16(SPI3);
/// - Write GPIOA ODR to release port
/// \return data byte read from address
uint8_t gyro_read(uint8_t address);

/// \brief Send gyro data to given address
/// \param uint8_t address
/// \param uint8_t data
/// Gyro is connected to SPI3, GPIOA, it is hard-coded in the function!
/// Operation:
/// - get status by calling SPI_GetReceptionFIFOStatus(SPI3)
/// - Wait until SPI_I2S_FLAG_TXE is set
/// - Send (address << 8)|data on SPI3
/// - Wait while SPI_GetReceptionFIFOStatus() is the same as initial
/// - Write SPI3->DR
/// - Write GPIOA ODR to release port
/// \return n return data
void gyro_send(uint8_t address, uint8_t data);

/// \brief Setup gyro by calling     gyro_send(GYRO_CTRL_REG2, 0x00)
/// and  gyro_send(GYRO_CTRL_REG1, 0x0F);
void gyro_setup();

/// \brief Print result of gyro_read(GYRO_OUT_TEMP);
void gyro_temp();

/// \brief Print result of gyro_read(GYRO_OUT_STAT;
void gyro_status();

/// \brief Print result of gyro_read(GYRO_OUT_WHOAMI);
void gyro_test();

/// \brief Print x,y,z values read from gyro.
/// ATTENTION: y value os offset by -150!!
void gyro_getValues();

/// \brief Init SPI3 with values defined in the course manual
void init_spi3();

#endif

#endif /* GYRO_H_INCLUDED */
