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

void init_accel();
//uint8_t gyro_temp();

/// \brief This function just reads the status of the accelerometer.
///         Uses \a accel_read_reg() \a function to read data from ACCEL_OUT_STAT
/// \param No parameter
void accel_status();


/// \brief This function reads the WHO_AM_I register of the accelerometer using the
///         \a accel_read_reg() \a function.
///
/// \param No parameter
void accel_test();


/// \brief This function reads and prints the accelerometer coordinates.
///         \a accel_read_reg() \a function.
/// \todo  Pay attention to the hard-coded values for adjusting displayed values.
///
/// \param No parameter
/// \return ((y-1700)/10);
int16_t accel_getValues();
#endif
