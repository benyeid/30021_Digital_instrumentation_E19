#include "gyro.h"
#include <inttypes.h>
#include "registers.h"

/*****************************/
/*** Gyro Control Functions ***/
/*****************************/
/*
uint8_t spi3_recv_byte() {
    int data;
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) {  }
    data = 0xFF & SPI_I2S_ReceiveData16(SPI3);
    return data;
}

void spi3_transmit_byte(uint8_t data) {
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }
    SPI_SendData8(SPI3, data);
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }
}

void spi3_transmit_word(uint16_t data) {
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) {  }
    SPI_I2S_SendData16(SPI3, data);
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }
}

*/
/*
void setup_gyro(){
    GPIOA->ODR &= ~(0x0001 << 4);           // PA_4 : CS = 0 - To start transmission
    send_data(CTRL_REG4 & 0x7F, 4);        ///Select CTRL_REG4 by sending the address & (enable write operation)
    send_data(0x80, 4);                        // BDU-1 (1: output registers not updated until MSB and LSB reading)
    send_data(CTRL_REG5 & 0x7F, 4);        ///Select CTRL_REG5 by sending the address & (enable write operation)
    send_data(0xC0, 4);                        // Reboot memory content & FIFO enable
    send_data(CTRL_REG1 & 0x7F, 4);        ///Select CTRL_REG1 by sending the address & (enable write operation)
    send_data(0xDF, 4);                        // Set power to normal mode and Enable x, y and z axis reading & Set Output Data Rate, Bandwidth
    GPIOA->ODR |=  (0x0001 << 4);        // PA_4 : CS = 1 - To end transmission
}*/

uint8_t gyro_read_reg(uint8_t addr)
{
    uint8_t data;
    addr |= 0x80;

    GYRO_CS_LOW();
    spi3_transmit_word(addr << 8);
    data = spi3_recv_byte();
    GYRO_CS_HIGH();

    return data;
}

void gyro_write_reg(uint8_t addr, uint8_t data)
{
    uint16_t initial_status = SPI_GetReceptionFIFOStatus(SPI3);
    addr &= 0x7f;

    GYRO_CS_LOW();
    spi3_transmit_word((addr<<8) | data);

    while (SPI_GetReceptionFIFOStatus(SPI3) == initial_status);
    SPI3->DR;

    GYRO_CS_HIGH();
}

void gyro_reset()
{
    GYRO_CS_HIGH();
    gyro_write_reg(GYRO_CTRL_REG2, 0x00);
    gyro_write_reg(GYRO_CTRL_REG1, 0x0F);
}

uint8_t gyro_temp()
{
    return gyro_read_reg(GYRO_OUT_TEMP);
}
/*
void Gyro_temp(){
    read_data(, 4);            //To load the address of output temperature register
    int temp_val = gyro_read(0xFF, 4); ///After loading address, sending a random value 0xFF to get the output temperature stored in output register
                                    ///Successful read is observed only after two Tx & Rx cycles
    printf("| Temperature > %d \n", temp_val); //To print the 8-bit temperature data in decimal
}*/


uint8_t gyro_status()
{
    return gyro_read_reg(GYRO_OUT_STAT);
}

uint8_t gyro_whoami()
{
    return gyro_read_reg(GYRO_WHOAMI);
}

void gyro_read(int16_t *x, int16_t *y, int16_t *z)
{
    *x = (gyro_read_reg(GYRO_OUT_X_H)<<8) | gyro_read_reg(GYRO_OUT_X_L);
    *y = (gyro_read_reg(GYRO_OUT_Y_H)<<8) | gyro_read_reg(GYRO_OUT_Y_L);
    *z = (gyro_read_reg(GYRO_OUT_Z_H)<<8) | gyro_read_reg(GYRO_OUT_Z_L);
}

void init_spi_gyro() {

    // Enable Clocks
    RCC->APB1ENR |= RCC_APB1Periph_SPI3;
    initPinAlternate(GPIOC, 10, 6);
    initPinAlternate(GPIOC, 11, 6);
    initPinAlternate(GPIOC, 12, 6);
    initPin(GPIOD, 2, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_RESET);

    // Configure SPI3
    SPI3->CR1 &= 0x3040; // Clear CR1 Register
    SPI3->CR1 |= 0x0000; // Configure direction (0x0000 - 2 Lines Full Duplex, 0x0400 - 2 Lines RX Only, 0x8000 - 1 Line RX, 0xC000 - 1 Line TX)
    SPI3->CR1 |= 0x0104; // Configure mode (0x0000 - Slave, 0x0104 - Master)
    SPI3->CR1 |= 0x0002; // Configure clock polarity (0x0000 - Low, 0x0002 - High)
    SPI3->CR1 |= 0x0001; // Configure clock phase (0x0000 - 1 Edge, 0x0001 - 2 Edge)
    SPI3->CR1 |= 0x0200; // Configure chip select (0x0000 - Hardware based, 0x0200 - Software based)
    SPI3->CR1 |= 0x0008; // Set Baud Rate Prescaler (0x0000 - 2, 0x0008 - 4, 0x0018 - 8, 0x0020 - 16, 0x0028 - 32, 0x0028 - 64, 0x0030 - 128, 0x0038 - 128)
    SPI3->CR1 |= 0x0000; // Set Bit Order (0x0000 - MSB First, 0x0080 - LSB First)
    SPI3->CR2 &= ~0x0F00; // Clear CR2 Register
    SPI3->CR2 |= 0x0F00; // Set Number of Bits (0x0300 - 4, 0x0400 - 5, 0x0500 - 6, ...);
    SPI3->I2SCFGR &= ~0x0800; // Disable I2S
    SPI3->CRCPR = 7; // Set CRC polynomial order
    SPI3->CR2 &= ~0x1000;
    SPI3->CR2 |= 0x1000; // Configure RXFIFO return at (0x0000 - Half-full (16 bits), 0x1000 - Quarter-full (8 bits))
    SPI3->CR1 |= 0x0040; // Enable SPI3

    for (uint32_t i = 0 ; i < 5000000 ; i++) { __asm__("nop"); }; // Wait
    gyro_reset();
}

void Gyro_test(){
    read_data(WHO_AM_I, 4);                    //To load the address of WHO_AM_I register
    uint8_t value = gyro_read(WHO_AM_I, 4);    ///To read the value stored in the register
    if(value == 0xD3){                     //To verify if the value is read correctly
        printf("\n Gyro is read correctly: %x  ", value); /// To print the reference value in hexadecimal
    }else{
        printf("\n Gyro is not read correctly ");
    }
}



