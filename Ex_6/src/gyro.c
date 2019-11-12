#include "gyro.h"
#include "spi.h"

/*****************************/
/*** Gyro Control Functions ***/
/*****************************/

#define GYRO_READ_MASK 0x80
#define GYRO_WRITE_MASK 0x7f

uint8_t gyro_read_reg(uint8_t addr)
{
    uint8_t data;
    addr |= GYRO_READ_MASK;

    GYRO_CS_LOW();
    spi3_transmit_word(addr << 8);
    data = spi3_recv_byte();
    GYRO_CS_HIGH();

    return data;
}

void gyro_write_reg(uint8_t addr, uint8_t data)
{
    uint16_t initial_status = SPI_GetReceptionFIFOStatus(SPI3);
    addr &= GYRO_WRITE_MASK;

    GYRO_CS_LOW();
    spi3_transmit_word((addr<<8) | data);
    GYRO_CS_HIGH();
}

void gyro_reset()
{
    GYRO_CS_HIGH();
    gyro_write_reg(CTRL_REG2, 0x00);
    gyro_write_reg(CTRL_REG1, 0x0F);
}

uint8_t gyro_temp()
{
    return gyro_read_reg(OUT_TEMP);
}

uint8_t gyro_status()
{
    return gyro_read_reg(GYRO_OUT_STAT);
}

uint8_t gyro_whoami()
{
    return gyro_read_reg(WHO_AM_I);
}

void gyro_read(int16_t *x, int16_t *y, int16_t *z)
{
    *x = (gyro_read_reg(OUT_X_H)<<8) | gyro_read_reg(OUT_X_L);
    *y = (gyro_read_reg(OUT_Y_H)<<8) | gyro_read_reg(OUT_Y_L);
    *z = (gyro_read_reg(OUT_Z_H)<<8) | gyro_read_reg(OUT_Z_L);
}

void init_gyro()
{
    // Init chip select
    initPin(GPIOD, 2, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_RESET);

    // Wait for initializations
    for (uint32_t i = 0 ; i < 5000000 ; i++) { __asm__("nop"); };
    gyro_reset();
}
