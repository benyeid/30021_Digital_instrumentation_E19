#include "accel.h"
#include "spi.h"

/*****************************/
/*** Accel Control Functions ***/
/*****************************/

uint8_t accel_read_reg(uint8_t addr)
{
    uint8_t data;
    addr |= 0x80;

    GPIOA->ODR &=~ (0x0001 << 15);
    spi3_transmit_word(addr << 8);
    data = spi3_recv_byte();
    GPIOA->ODR |= (0x0001 << 15);

    return data;
}

void accel_write_reg(uint8_t addr, uint8_t data)
{
    uint16_t initial_status = SPI_GetReceptionFIFOStatus(SPI3);
    addr &= 0x7f;

    GPIOA->ODR &=~ (0x0001 << 15);

    spi3_transmit_word((addr<<8) | data);

    while (SPI_GetReceptionFIFOStatus(SPI3) == initial_status);
    SPI3->DR;

    GPIOA->ODR |= (0x0001 << 15);
}

void accel_setup()
{
    accel_write_reg(ACCEL_CTRL_REG4, 0x03);
    accel_write_reg(ACCEL_CTRL_REG1, 0x67);
}

void accel_status()
{
    uint8_t data = accel_read_reg(ACCEL_OUT_STAT);
    printf("ACCEL_OUT_STATUS = %02x \n", data);
}

void accel_test()
{
    uint8_t data = accel_read_reg(ACCEL_WHOAMI);
    printf("Accel ID = %x \n", data);
}

int16_t accel_getValues()
{
    int16_t x, y, z = 0;
    x = (accel_read_reg(ACCEL_OUT_X_H)<<8) | accel_read_reg(ACCEL_OUT_X_L);
    y = (accel_read_reg(ACCEL_OUT_Y_H)<<8) | accel_read_reg(ACCEL_OUT_Y_L);
    z = (accel_read_reg(ACCEL_OUT_Z_H)<<8) | accel_read_reg(ACCEL_OUT_Z_L);
    printf("Accel || x : y : z ||  %d  :  %d  :  %d  \n", (x-16600)/10 , (y-1700)/10, (z-700)/10);

    return ((y-1700)/10);
}

