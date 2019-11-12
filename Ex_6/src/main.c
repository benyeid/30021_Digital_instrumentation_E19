
#include "stm32f30x_conf.h"
#include "30021_io.h"
#include "gyro.h"
#include "accelerometer.h"
#include "magnetometer.h"
#include "spi.h"
#include <stdio.h>

int main(void)
{
    int16_t x,y,z;
    uint8_t id, status;
    init_usb_uart(9600);
    spi3_init();
    init_Acc();
    init_mag();
    id = Acc_whoami();

    x = y = z = 0;
    while(1)
    {
	status = Acc_status();
	Acc_read(&x, &y, &z);

	printf("id: %02x, status: %02x\t", id, status);
	printf("x: %d, y: %d, z: %d\n", x, y, z);
	for(int i = 0; i < 100; ++i);
    }
}


/*
#include "stm32f30x_conf.h"
#include "30021_io.h"
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "lcd.h"
#include <time.h>
#include <inttypes.h>
//#include "registers.h"

void init_spi3() {
    // Enable Clocks

    RCC->AHBENR  |= 0x00020000 | 0x00040000;    // Enable Clock for GPIO Banks A and B
    RCC->APB1ENR |= 0x00008000;                 // Enable Clock for SPI3

    // Connect pins to SPI3
    GPIOB->AFR[3 >> 0x03] &= ~(0x0000000F << ((3 & 0x00000007) * 4)); // Clear alternate function for PB3
    GPIOB->AFR[3 >> 0x03] |=  (0x00000006 << ((3 & 0x00000007) * 4)); // Set alternate 6 function for PB3 - SCLK
    GPIOB->AFR[5 >> 0x03] &= ~(0x0000000F << ((5 & 0x00000007) * 4)); // Clear alternate function for PB5
    GPIOB->AFR[5 >> 0x03] |=  (0x00000006 << ((5 & 0x00000007) * 4)); // Set alternate 6 function for PB5 - MOSI
    GPIOB->AFR[4 >> 0x03] &= ~(0x0000000F << ((4 & 0x00000007) * 4)); // Clear alternate function for PB4
    GPIOB->AFR[4 >> 0x03] |=  (0x00000006 << ((4 & 0x00000007) * 4)); // Set alternate 6 function for PB4 - MISO

    /// Configure pins PB3 (SCLK) and PB5(MOSI) for 10 MHz alternate function
    GPIOB->OSPEEDR &= ~(0x00000003 << (3 * 2) | 0x00000003 << (5 * 2));    // Clear speed register
    GPIOB->OSPEEDR |=  (0x00000001 << (3 * 2) | 0x00000001 << (5 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOB->OTYPER  &= ~(0x0001     << (3)     | 0x0001     << (5));        // Clear output type register
    GPIOB->OTYPER  |=  (0x0000     << (3)     | 0x0000     << (5));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOB->MODER   &= ~(0x00000003 << (3 * 2) | 0x00000003 << (5 * 2));    // Clear mode register
    GPIOB->MODER   |=  (0x00000002 << (3 * 2) | 0x00000002 << (5 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOB->PUPDR   &= ~(0x00000003 << (3 * 2) | 0x00000003 << (5 * 2));    // Clear push/pull register
    GPIOB->PUPDR   |=  (0x00000000 << (3 * 2) | 0x00000000 << (5 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    /// Configurate PB4(MISO) for 10MHz alternate function
    GPIOB->OSPEEDR &= ~(0x00000003 << (4 * 2));    // Clear speed register
    GPIOB->OSPEEDR |=  (0x00000001 << (4 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOB->OTYPER  &= ~(0x0001     << (4));        // Clear output type register
    GPIOB->OTYPER  |=  (0x0000     << (4));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOB->MODER   &= ~(0x00000003 << (4 * 2));    // Clear mode register
    GPIOB->MODER   |=  (0x00000002 << (4 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOB->PUPDR   &= ~(0x00000003 << (4 * 2));    // Clear push/pull register
    GPIOB->PUPDR   |=  (0x00000000 << (4 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    ///Set PA4: CS for gyro and PA14: CS for Acc and PA15: CS for Magnetometer
    GPIOA->OSPEEDR &= ~(0x00000003 << (4 * 2)) | (0x00000003 << (15 * 2))| (0x00000003 << (14 * 2));
    GPIOA->OSPEEDR |=  (0x00000001 << (4 * 2)) | (0x00000001 << (15 * 2))| (0x00000001 << (14 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOA->OTYPER  &= ~(0x0001     << (4))     | (0x0001     << (15))    | (0x0001 << (14 * 2));        // Clear output type register
    GPIOA->OTYPER  |=  (0x0000     << (4))     | (0x0000     << (15))    | (0x0000 << (14 * 2));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOA->MODER   &= ~(0x00000003 << (4 * 2)) | (0x00000003 << (15 * 2))| (0x00000003 << (14 * 2));    // Clear mode register
    GPIOA->MODER   |=  (0x00000001 << (4 * 2)) | (0x00000001 << (15 * 2))| (0x00000001 << (14 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOA->PUPDR   &= ~(0x00000003 << (4 * 2)) | (0x00000003 << (15 * 2))| (0x00000003 << (14 * 2));    // Clear push/pull register
    GPIOA->PUPDR   |=  (0x00000000 << (4 * 2)) | (0x00000000 << (15 * 2))| (0x00000000 << (14 * 2));
    GPIOA->ODR |=  (0x0001 << 4)               | (0x0001 << 15)          | (0x0001 << 14); // CS = 1

    /// Configure SPI3
    SPI3->CR1 &= 0x3040; // Clear CR1 Register
    SPI3->CR1 |= 0x0000; // Configure direction (0x0000 - 2 Lines Full Duplex, 0x0400 - 2 Lines RX Only, 0x8000 - 1 Line RX, 0xC000 - 1 Line TX)
    SPI3->CR1 |= 0x0104; // Configure mode (0x0000 - Slave, 0x0104 - Master)
    SPI3->CR1 |= 0x0002; // Configure clock polarity (0x0000 - Low, 0x0002 - High)
    SPI3->CR1 |= 0x0001; // Configure clock phase (0x0000 - 1 Edge, 0x0001 - 2 Edge)
    SPI3->CR1 |= 0x0200; // Configure chip select (0x0000 - Hardware based, 0x0200 - Software based)
    SPI3->CR1 |= 0x0028; // Set Baud Rate Prescaler (0x0000 - 2, 0x0008 - 4, 0x0018 - 8, 0x0020 - 16, 0x0028 - 32, 0x0028 - 64, 0x0030 - 128, 0x0038 - 128)
    SPI3->CR1 |= 0x0000; // Set Bit Order (0x0000 - MSB First, 0x0080 - LSB First)
    SPI3->CR2 &= ~0x0F00; // Clear CR2 Register
    SPI3->CR2 |= 0x0700; // Set Number of Bits (0x0300 - 4, 0x0400 - 5, 0x0500 - 6, ...);
    SPI3->I2SCFGR &= ~0x0800; // Disable I2S
    SPI3->CRCPR = 7; // Set CRC polynomial order
    SPI3->CR2 &= ~0x1000;
    SPI3->CR1 |= 0x0040; // Enable SPI3
    SPI3->CR1 |= 0b101000;
}

uint8_t read_data(uint8_t data,uint8_t cs){
    data |= 0x80;
    if (cs == 15 || cs == 14){
        SPI3->CR2 &=~0x0000;
        SPI3->CR2 |= 0x0000;
    } else if (cs == 4){
        SPI3->CR2 &=~0x1000;
        SPI3->CR2 |= 0x1000;
    }
    GPIOA->ODR &= ~(0x0001 << cs);           /// PA_15 : CS = 0 - to Start Transmission to Mag
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }
    SPI_SendData8(SPI3, data);              // Data sent to SPI serial data input while Tx is enabled
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { }
    uint8_t byte = SPI_ReceiveData8(SPI3);  /// Data read from SPI serial data output after completion of Tx & while Rx is enabled
    GPIOA->ODR |=  (0x0001 << cs);           // PA_4 : CS = 1 - End Transmission
	return byte;
}

void send_data(uint8_t data, uint8_t cs){
   // uint16_t check_FIFO = SPI_GetReceptionFIFOStatus(SPI3); //might be needed for reading position etc.

    data &= 0x7F;
	if (cs == 15 || cs == 14){
    SPI3->CR2 &=~0x0000;
    SPI3->CR2 |= 0x0000;
  } else if (cs == 4){
    SPI3->CR2 &=~0x1000;
    SPI3->CR2 |= 0x1000;
  }
    GPIOA->ODR |= (0x0001 << cs);
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }
    SPI_SendData8(SPI3, data);
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }

    //while (SPI_GetReceptionFIFOStatus(SPI3) == initial_status); //might be needed for reading position etc.
    //SPI3->DR;                                                   //that goes together with above line

    GPIOA->ODR |= (0x0001 << cs);
}

void setup_gyro(){
    GPIOA->ODR &= ~(0x0001 << 4);           // PA_4 : CS = 0 - To start transmission
    send_data(CTRL_REG4 & 0x7F, 4);        ///Select CTRL_REG4 by sending the address & (enable write operation)
    send_data(0x80, 4);                        // BDU-1 (1: output registers not updated until MSB and LSB reading)
    send_data(CTRL_REG5 & 0x7F, 4);        ///Select CTRL_REG5 by sending the address & (enable write operation)
    send_data(0xC0, 4);                        // Reboot memory content & FIFO enable
    send_data(CTRL_REG1 & 0x7F, 4);        ///Select CTRL_REG1 by sending the address & (enable write operation)
    send_data(0xDF, 4);                        // Set power to normal mode and Enable x, y and z axis reading & Set Output Data Rate, Bandwidth
    GPIOA->ODR |=  (0x0001 << 4);        // PA_4 : CS = 1 - To end transmission
}

void setup_Mag(){
    GPIOA->ODR &= ~(0x0001 << 15);           // PA_4 : CS = 0 - To start transmission
    send_data(CTRL_REG3_M & 0x7F, 15);
    send_data(0x85, 15);
    send_data(CTRL_REG4_M & 0x7F, 15);        ///Select CTRL_REG4 by sending the address & (enable write operation)
    send_data(0x80, 15);                        // BDU-1 (1: output registers not updated until MSB and LSB reading)
    send_data(CTRL_REG5_M & 0x7F, 15);        ///Select CTRL_REG5 by sending the address & (enable write operation)
    send_data(0xC0, 15);                        // Reboot memory content & FIFO enable
    send_data(CTRL_REG1_M & 0x7F, 15);        ///Select CTRL_REG1 by sending the address & (enable write operation)
    send_data(0xDF, 15);                        // Set power to normal mode and Enable x, y and z axis reading & Set Output Data Rate, Bandwidth
    GPIOA->ODR |=  (0x0001 << 15);        // PA_4 : CS = 1 - To end transmission
}

void setup_Acc(){
    GPIOA->ODR &=~(0x0001 << 14);
    send_data(CTRL_REG4_A & 0x7F, 14);
    send_data(0x03, 15);
    send_data(CTRL_REG1_A & 0x7F, 14);        ///Select CTRL_REG4 by sending the address & (enable write operation)
    send_data(0x67, 15);
    GPIOA->ODR |= (0x0001 << 14);
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

void Gyro_temp(){
    read_data(OUT_TEMP, 4);            //To load the address of output temperature register
    int temp_val = gyro_read(0xFF, 4); ///After loading address, sending a random value 0xFF to get the output temperature stored in output register
                                    ///Successful read is observed only after two Tx & Rx cycles
    printf("| Temperature > %d \n", temp_val); //To print the 8-bit temperature data in decimal
}

void Mag_test(){
    read_data(WHO_AM_I_M,15);                    //To load the address of WHO_AM_I register
    uint8_t value = read_data(WHO_AM_I_M,15);    ///To read the value stored in the register
    if(value == 0x3D){                      //To verify if the value is read correctly
        printf("\n Magnetic sensor is read correctly: %x  ", value); /// To print the reference value in hexadecimal
    }else{
        printf("\n Mag is not read correctly ");
    }
}

void Acc_test(){
    read_data(WHO_AM_I_A, 14);                    //To load the address of WHO_AM_I register
    uint8_t value = read_data(WHO_AM_I_A, 14);    ///To read the value stored in the register
    if(value == 0x41){                       //To verify if the value is read correctly
        printf("\n Accelerometer is read correctly: %x  ", value); /// To print the reference value in hexadecimal
    }else{
        printf("\n Acc is not read correctly ");
    }
}

void getGyroValues(){

uint16_t *x = 0;
uint16_t *y = 0;
uint16_t *z = 0;

    read_data(OUT_X_H | 0x40, 14);    //To load the address OUT_X_H where the higher order 8-bits are stored
    *x = read_data((0xFF)<<8, 14);       ///To read the data from the register, a random value 0xFF is sent
    read_data(OUT_X_L | 0x40, 14);    //To load the address OUT_X_L where the lower order 8-bits are stored
    *x |= read_data(0xFF, 14);             ///To read the data from the register, a random value 0xFF is sent

    read_data(OUT_Y_H | 0x40, 14);      //Similar process is repeated for the y and z axes
    *y = read_data((0xFF)<<8, 14);
    read_data(OUT_Y_L | 0x40, 14);
    *y |= read_data(0xFF, 14);

    read_data(OUT_Z_H | 0x40, 14);
    *z = read_data((0xFF)<<8, 14);
    read_data(OUT_Z_L | 0x40, 14);
    *z |= (read_data(0xFF, 14));
    printf("Gyros' values: x: %d, y: %d, z: %d\t", x, y, z);
}

void getAccValues(){

    uint16_t *x = 0;
    uint16_t *y = 0;
    uint16_t *z = 0;
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
}

void getMagValues(){

    uint16_t *x = 0;
    uint16_t *y = 0;
    uint16_t *z = 0;
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
}

int main(void){
    init_usb_uart(9600);
    init_spi3();
printf("a");
    setup_Mag();
    Mag_test();

    while(1)
    {
        getMagValues();
    }
}*/
