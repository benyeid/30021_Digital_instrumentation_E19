#include "accelerometer.h"
#include "spi.h"


#define ACC_READ_MASK 0x80
#define ACC_WRITE_MASK 0x7f
/*
void setup_Acc(){
    GPIOA->ODR &=~(0x0001 << 14);

    //TODO send it with original send_data function??
    Mag_send(CTRL_REG4_A & 0x7F, 14);
    Mag_send(0x03, 15);
    Mag_send(CTRL_REG1_A & 0x7F, 14);        ///Select CTRL_REG4 by sending the address & (enable write operation)
    Mag_send(0x67, 15);
    GPIOA->ODR |= (0x0001 << 14);
}*/

uint8_t Acc_ReadRegister(uint8_t addr)
{
    uint8_t data;
    addr |= ACC_READ_MASK;

    ACCEL_CS_LOW();
    spi3_transmit_word(addr << 8);
    data = spi3_recv_byte();
    ACCEL_CS_HIGH();

    return data;
}

void Acc_WriteReg(uint8_t addr, uint8_t data)
{
    uint16_t initial_status = SPI_GetReceptionFIFOStatus(SPI3);
    addr &= ACC_WRITE_MASK;

    ACCEL_CS_LOW();
    spi3_transmit_word((addr<<8) | data);

    while (SPI_GetReceptionFIFOStatus(SPI3) == initial_status);
    SPI3->DR;

    ACCEL_CS_HIGH();
}

void Acc_reset()
{
    ACCEL_CS_HIGH();
    Acc_WriteReg(CTRL_REG4_A, 0x03);
    Acc_WriteReg(CTRL_REG4_A, 0x67);
}

uint8_t Acc_status()
{
    return Acc_ReadRegister(ACC_STATUS_REG);
}

uint8_t Acc_whoami()
{
    return Acc_ReadRegister(WHO_AM_I_A);
}

void Acc_read(int16_t *x, int16_t *y, int16_t *z)
{
    *x = (Acc_ReadRegister(OUT_X_H_A)<<8) | Acc_ReadRegister(OUT_X_L_A);
    *y = (Acc_ReadRegister(OUT_Y_H_A)<<8) | Acc_ReadRegister(OUT_Y_L_A);
    *z = (Acc_ReadRegister(OUT_Z_H_A)<<8) | Acc_ReadRegister(OUT_Z_L_A);
}


void init_Acc()
{
    // Init chip select
    initPin(ACCEL_CS_GPIO, ACCEL_CS_PIN, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_RESET);

    // Wait for initializations
    //for (uint32_t i = 0 ; i < 5000000 ; i++) { __asm__("nop"); };
    Acc_reset();
}




void Acc_test(){

	Acc_ReadRegister(14);
    //read_data(WHO_AM_I_A, 14);                    //To load the address of WHO_AM_I register
    uint8_t value =Acc_ReadRegister(14);// read_data(WHO_AM_I_A, 14);    ///To read the value stored in the register
    if(value == 0x41){                       //To verify if the value is read correctly
        printf("\n Accelerometer is read correctly: %x  ", value); /// To print the reference value in hexadecimal
    }else{
        printf("\n Acc is not read correctly ");
    }
}
/*
void getAccValues(uint16_t *x, uint16_t *y, uint16_t *z){
    read_data(OUT_X_H_A | 0x40, 14);    //To load the address OUT_X_H where the higher order 8-bits are stored
    *x = read_data((0xFF)<<8, 14);       ///To read the data from the register, a random value 0xFF is sent
    read_data(OUT_X_L_A | 0x40, 14);    //To load the address OUT_X_L where the lower order 8-bits are stored
    *x |= read_data(0xFF, 14);             ///To read the data from the register, a random value 0xFF is sent

    read_data(OUT_Y_H_A | 0x40, 14);      //Similar process is repeated for the y and z axes
    *y = read_data((0xFF)<<8, 14);
    read_data(OUT_Y_L_A | 0x40, 14);
    *y |= read_data(0xFF, 14);

    read_data(OUT_Z_H_A | 0x40, 14);
    *z = read_data((0xFF)<<8, 14);
    read_data(OUT_Z_L_A | 0x40, 14);
    *z |= (read_data(0xFF, 14));
    printf("Accelerometers' values: x: %d, y: %d, z: %d\t", x, y, z);
}*/
