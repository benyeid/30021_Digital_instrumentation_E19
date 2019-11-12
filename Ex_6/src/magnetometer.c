#include "magnetometer.h"
#include "spi.h"
#include "registers.h"

#define MAG_READ_MASK 0x80
#define MAG_WRITE_MASK 0x7f
/*
void setup_Mag(){
    GPIOA->ODR &= ~(0x0001 << 15);           // PA_4 : CS = 0 - To start transmission
    Mag_send(CTRL_REG3_M & 0x7F, 15);
    Mag_send(0x85, 15);
    Mag_send(CTRL_REG4_M & 0x7F, 15);        ///Select CTRL_REG4 by sending the address & (enable write operation)
    Mag_send(0x80, 15);                        // BDU-1 (1: output registers not updated until MSB and LSB reading)
    Mag_send(CTRL_REG5_M & 0x7F, 15);        ///Select CTRL_REG5 by sending the address & (enable write operation)
    Mag_send(0xC0, 15);                        // Reboot memory content & FIFO enable
    Mag_send(CTRL_REG1_M & 0x7F, 15);        ///Select CTRL_REG1 by sending the address & (enable write operation)
    Mag_send(0xDF, 15);                        // Set power to normal mode and Enable x, y and z axis reading & Set Output Data Rate, Bandwidth
    GPIOA->ODR |=  (0x0001 << 15);        // PA_4 : CS = 1 - To end transmission
}*/

uint8_t MagReadRegisters(uint8_t addr)
{
    uint8_t data;
    addr |= MAG_READ_MASK;

    MAG_CS_LOW();
    spi3_transmit_word(addr << 8);
    data = spi3_recv_byte();
    MAG_CS_HIGH();

    return data;
}

void MagWriteRegisters(uint8_t addr, uint8_t data)
{
    uint16_t initial_status = SPI_GetReceptionFIFOStatus(SPI3);
    addr &= MAG_WRITE_MASK;

    MAG_CS_LOW();
    spi3_transmit_word((addr<<8) | data);

    while (SPI_GetReceptionFIFOStatus(SPI3) == initial_status);
    SPI3->DR;

    MAG_CS_HIGH();
}

void Mag_Reset()
{
    MAG_CS_HIGH();
    MagWriteRegisters(CTRL_REG3_M, 0x85);
    //mag_write_reg(MAG_CTRL_REG1, 0x67);
}

uint8_t Mag_Status()
{
    return MagReadRegisters(MAG_OUT_STAT);
}

uint8_t Mag_Whoami()
{
    return MagReadRegisters(WHO_AM_I_M);
}

void mag_read(int16_t *x, int16_t *y, int16_t *z)
{
    *x = (MagReadRegisters(OUT_X_H_M)<<8) | MagReadRegisters(OUT_X_L_M);
    *y = (MagReadRegisters(OUT_Y_H_M)<<8) | MagReadRegisters(OUT_Y_L_M);
    *z = (MagReadRegisters(OUT_Z_H_M)<<8) | MagReadRegisters(OUT_Z_L_M);
}


void init_mag()
{
    // Init chip select
    initPin(MAG_CS_GPIO, MAG_CS_PIN, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_RESET);

    // Wait for initializations
    //for (uint32_t i = 0 ; i < 5000000 ; i++) { __asm__("nop"); };
    Mag_Reset();
}

/*
void Mag_test(){
    read_data(WHO_AM_I_M,15);                    //To load the address of WHO_AM_I register
    uint8_t value = read_data(WHO_AM_I_M,15);    ///To read the value stored in the register
    if(value == 0x3D){                      //To verify if the value is read correctly
        printf("\n Magnetic sensor is read correctly: %x  ", value); /// To print the reference value in hexadecimal
    }else{
        printf("\n Mag is not read correctly ");
    }
}

void getMagValues(uint16_t *x, uint16_t *y, uint16_t *z){
    read_data(OUT_X_H_M | 0x40, 15);    //To load the address OUT_X_H where the higher order 8-bits are stored
    *x = read_data((0xFF)<<8, 15);       ///To read the data from the register, a random value 0xFF is sent
    read_data(OUT_X_L_M | 0x40, 15);    //To load the address OUT_X_L where the lower order 8-bits are stored
    *x |= read_data(0xFF, 15);             ///To read the data from the register, a random value 0xFF is sent

    read_data(OUT_Y_H_M | 0x40, 15);      //Similar process is repeated for the y and z axes
    *y = read_data((0xFF)<<8, 15);
    read_data(OUT_Y_L_M | 0x40, 15);
    *y |= read_data(0xFF, 15);

    read_data(OUT_Z_H_M | 0x40, 15);
    *z = read_data((0xFF)<<8, 15);
    read_data(OUT_Z_L_M | 0x40, 15);
    *z |= (read_data(0xFF, 15));
    printf("Magnetometers' values: x: %d, y: %d, z: %d\t", x, y, z);
}*/
