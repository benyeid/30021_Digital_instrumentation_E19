#ifndef __MAG_H
#define __MAG_H
#include "stm32f30x_conf.h"
#include "gpio.h"

#define MAG_CTRL_REG1 0x20
#define MAG_CTRL_REG2 0x21
#define MAG_CTRL_REG3 0x22
#define MAG_CTRL_REG4 0x23
#define MAG_OUT_STAT 0x27
#define MAG_OUT_X_L 0x28
#define MAG_OUT_X_H 0x29
#define MAG_OUT_Y_L 0x2A
#define MAG_OUT_Y_H 0x2B
#define MAG_OUT_Z_L 0x2C
#define MAG_OUT_Z_H 0x2D
#define MAG_WHOAMI 0x0F

///\brief Read data from the register given by parameter
/// Depends on spi3_transmit_word()
///
/// \param uint8_t Register address
/// \return uint8_t Data in he register
uint8_t mag_read_reg(uint8_t addr);

/// \brief Write given data to given register
/// Depends on spi3_transmit_word()
///
/// \param uint8_t Register address, uint8_t data to write
/// \return Void function, no return value
void mag_write_reg(uint8_t addr, uint8_t data);

/// \brief Setup magnetometer by writing value 0x85 to MAG_CTRL_REG3
/// and 0x67 to MAG_CTRL_REG1.
/// Depends on mag_write_reg();
void mag_setup()

/// \todo Write this function based on void mag_getValues(); ASAP
void mag_read(int16_t *x, int16_t *y, int16_t *z);
void init_mag();
//uint8_t gyro_temp();

/// \brief Get status of the magnetometer by calling mag_read_reg(MAG_OUT_STAT);
void mag_status();

/// \brief Basic ping and status of magnetometer by calling mag_read_reg(MAG_WHOAMI);
void mag_whoami();

/// \brief Prints x,y,z coordinate values of read from magnetometer to console
void mag_getValues();
#endif
