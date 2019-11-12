#include "../inc/30021_io.h"
#include "flash.h"
#include "lcd.h"
#include "stm32f30x_conf.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "accelerometer.h"
#include "magnetometer.h"
#include "registers.h"


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
    uint16_t check_FIFO = SPI_GetReceptionFIFOStatus(SPI3); //might be needed for reading position etc.

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

    while (SPI_GetReceptionFIFOStatus(SPI3) == initial_status); //might be needed for reading position etc.
    SPI3->DR;                                                   //that goes together with above line

    GPIOA->ODR |= (0x0001 << cs);
}


/*
uint8_t read_data(uint8_t data, uint8_t cs) {
  SPI3->CR2 &= ~0x0000;
  SPI3->CR2 |= 0x0000; // Acc and Mag configurate RXFIFO as quarter-full

  data |= 0x80; // To enable read from Magnetic sensor
  GPIOA->ODR &=
      ~(0x0001 << cs); /// PA_15 : CS = 0 - to Start Transmission to Mag

  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) {
  }

  // SPI3->DR = data;
  SPI_SendData8(SPI3,
                data); // Data sent to SPI serial data input while Tx is enabled

  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) {
  }

  // return (uint8_t)((SPI3->DR));
  uint8_t byte =
      SPI_ReceiveData8(SPI3); /// Data read from SPI serial data output after
                              /// completion of Tx & while Rx is enabled

  GPIOA->ODR |= (0x0001 << cs); // PA_4 : CS = 1 - End Transmission
  return byte;
}*/

uint8_t Mag_send(uint8_t val) {
  SPI3->CR2 &= ~0x0000;
  SPI3->CR2 |= 0x0000; // Acc and Mag configurate RXFIFO as quarter-full

  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) {
  }

  SPI_SendData8(
      SPI3,
      val); // Address/data sent to SPI serial data input while Tx is enabled

  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) {
  }

  return 0;
  // while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) { }
  // uint8_t byte = SPI_ReceiveData8(SPI3);  // Data read from SPI serial data
  // output after completion of Tx & while Rx is enabled return byte;
}
/*
void transmit_byte_spi3(uint8_t data)
{
    data &= 0x7F;
    SPI3->DR = data;
    GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be
corrected

    SPI_SendData8(SPI3, data);

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) { } //need to be
corrected GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
}

uint8_t read_byte_spi3(uint8_t read){
    read |= 0x80;
    SPI3->DR = read;
    GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { } //need to
be corrected

    uint8_t byte = SPI_ReceiveData8(SPI3);

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) { } //need to be
corrected GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
return byte;
}


void transmit_string_spi3(const char* str, uint8_t lenght){
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be
corrected GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    for (int n = 0; n < lenght; n++)
        {
            uint8_t data = str[n];
            SPI_SendData8(SPI3, data);
        }
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be
corrected GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
}
*/



void Mag_test() {
  read_data(WHO_AM_I_M, 15); // To load the address of WHO_AM_I register
  uint8_t value =
      read_data(WHO_AM_I_M, 15); /// To read the value stored in the register

  if (value == 0x3D) // To verify if the value is read correctly
  {
    printf("\n Magnetic sensor is read correctly: %d  ",
           value); /// To print the reference value in hexadecimal
  } else {
    printf("\n Mag is not read correctly ");
  }
}
void Acc_test() {
  read_data(WHO_AM_I_A, 4); // To load the address of WHO_AM_I register
  uint8_t value =
      read_data(WHO_AM_I_A, 4); /// To read the value stored in the register

  if (value == 0x41) // To verify if the value is read correctly
  {
    printf("\n Accelerometer is read correctly: %d  ",
           value); /// To print the reference value in hexadecimal
  } else {
    printf("\n Acc is not read correctly ");
  }
}



/*
uint16_t getGyroValues(uint8_t axis)
{
    uint8_t cmd = (GYRO_READ_DATA | GYRO_MULTIPLE | axis);
    uint8_t result[2];  // [0] holds low byte, [1] holds high byte
    gyro_cs(CS_ACTIVE);
    gyro_read(cmd);    // Reads out 2 bytes from the gyro registers by applying
the GYRO_MULTIPLE in the read command, and then transmitting as many dummy bytes
as we want to read out. result[0] = gyro_read(0xFF); result[1] =
gyro_read(0xFF)); gyro_cs(CS_IDLE); return ((result[1] << 8) | result[0]);
}
*/

/// The x, y & z outputs of Gyrometer are received in 2's complement form which
/// are converted to decimal form using this function
void convert_to_decimal(uint16_t twos_complement) {
  int signed_integer;
  if (twos_complement & 0b1000000000000000) {
    signed_integer =
        twos_complement | ~(0b0111111111111111); // If the number is -ve
  } else {
    signed_integer = (~(twos_complement) |
                      0b0000000000000001); // If the number is in original form
  }
  return signed_integer;
}

int main(void) {
  init_usb_uart(9600); // Initialize UART
  init_spi3(); /// Initialize SPI_3, which uses CLK: PB_3, MISO: PB_4, MOSI:
               /// PB_5, CS: PA_4

  //setup_Mag(); // Setup gyro to read the X Y Z angular orientation
  // Mag_test();                          ///Test gyro if it reads WHO_AM_I
  // register value(Performed without setup_gyro();) gyro_temp_val(); //Test
  // gyro if it reads temperature (Performed without setup_gyro();)

  while (1) {
    //getMagValues();
    printf("sbsdgsd\n");
    /*
    gyro_read(STATUS_REG); // To load the address of Status
    register int status = gyro_read(0xFF);
    if (status | 0x0F) {
      getGyroValues();
    }*/
  }
}
