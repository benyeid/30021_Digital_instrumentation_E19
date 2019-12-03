#include "mag.h"
#include "spi.h"

/*****************************/
/*** Mag Control Functions ***/
/*****************************/

uint8_t mag_read_reg(uint8_t addr)
{
    uint8_t data;
    addr |= 0x80;

    GPIOA->ODR &=~ (0x0001 << 4);
    spi3_transmit_word(addr << 8);
    data = spi3_recv_byte();
    GPIOA->ODR &=~ (0x0001 << 4);

    return data;
}

void mag_write_reg(uint8_t addr, uint8_t data)
{
    uint16_t initial_status = SPI_GetReceptionFIFOStatus(SPI3);
    addr &= 0x7f;

    GPIOA->ODR &=~ (0x0001 << 4);
    spi3_transmit_word((addr<<8) | data);

    while (SPI_GetReceptionFIFOStatus(SPI3) == initial_status);
    SPI3->DR;

    GPIOA->ODR |= (0x0001 << 4);
}

void mag_setup()
{
    mag_write_reg(MAG_CTRL_REG3, 0x85);
    mag_write_reg(MAG_CTRL_REG1, 0x67);
}

void mag_status()
{
    uint8_t data = mag_read_reg(MAG_OUT_STAT);
    printf("MAG_STATUS = %02x \n", data);
}

void mag_whoami()
{
    uint8_t data = mag_read_reg(MAG_WHOAMI);
    printf("MAG WHO_AM_I = %02x \n", data);
}


void mag_getValues()
{
    int16_t x, y, z = 0;
    x = (mag_read_reg(MAG_OUT_X_H)<<8) | mag_read_reg(MAG_OUT_X_L);
    y = (mag_read_reg(MAG_OUT_Y_H)<<8) | mag_read_reg(MAG_OUT_Y_L);
    z = (mag_read_reg(MAG_OUT_Z_H)<<8) | mag_read_reg(MAG_OUT_Z_L);
    printf("x: %d, y: %d, z: %d\n", x, y, z);
}
