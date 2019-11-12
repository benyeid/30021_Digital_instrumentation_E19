#include "stm32f30x_conf.h"
#include "30021_io.h"
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "lcd.h"
#include <time.h>
#include <inttypes.h>

#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38

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

    ///setting PA4 (the CS)
    GPIOA->OSPEEDR &= ~(0x00000003 << (4 * 2));
    GPIOA->OSPEEDR |=  (0x00000001 << (4 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOA->OTYPER  &= ~(0x0001     << (4));        // Clear output type register
    GPIOA->OTYPER  |=  (0x0000     << (4));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOA->MODER   &= ~(0x00000003 << (4 * 2));    // Clear mode register
    GPIOA->MODER   |=  (0x00000001 << (4 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOA->PUPDR   &= ~(0x00000003 << (4 * 2));    // Clear push/pull register
    GPIOA->PUPDR   |=  (0x00000000 << (4 * 2));
    GPIOA->ODR |=  (0x0001 << 4); // CS = 1

    // Configure SPI3
    SPI3->CR1 &= 0x3040; // Clear CR1 Register
    SPI3->CR1 |= 0x0000; // Configure direction (0x0000 - 2 Lines Full Duplex, 0x0400 - 2 Lines RX Only, 0x8000 - 1 Line RX, 0xC000 - 1 Line TX)
    SPI3->CR1 |= 0x0104; // Configure mode (0x0000 - Slave, 0x0104 - Master)
    SPI3->CR1 |= 0x0002; // Configure clock polarity (0x0000 - Low, 0x0002 - High)
    SPI3->CR1 |= 0x0001; // Configure clock phase (0x0000 - 1 Edge, 0x0001 - 2 Edge)
    SPI3->CR1 |= 0x0200; // Configure chip select (0x0000 - Hardware based, 0x0200 - Software based)
    SPI3->CR1 |= 0x0000; // Set Baud Rate Prescaler (0x0000 - 2, 0x0008 - 4, 0x0018 - 8, 0x0020 - 16, 0x0028 - 32, 0x0028 - 64, 0x0030 - 128, 0x0038 - 128)
    SPI3->CR1 |= 0x0000; // Set Bit Order (0x0000 - MSB First, 0x0080 - LSB First)
    SPI3->CR2 &= ~0x0F00; // Clear CR2 Register
    SPI3->CR2 |= 0x0700; // Set Number of Bits (0x0300 - 4, 0x0400 - 5, 0x0500 - 6, ...);
    SPI3->I2SCFGR &= ~0x0800; // Disable I2S
    SPI3->CRCPR = 7; // Set CRC polynomial order
    SPI3->CR2 &= ~0x1000;
    SPI3->CR2 |= 0x1000; // Configure RXFIFO return at (0x0000 - Half-full (16 bits), 0x1000 - Quarter-full (8 bits))
    SPI3->CR1 |= 0x0040; // Enable SPI3
    SPI3->CR1 |= 0b111000;
}

uint8_t gyro_read(uint8_t data) {
    data |= 0x80;                           // To enable read from gyro
    GPIOA->ODR &= ~(0x0001 << 4);           /// PA_4 : CS = 0 - to Start Transmission

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }

    SPI_SendData8(SPI3, data);              // Data sent to SPI serial data input while Tx is enabled

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { }

    uint8_t byte = SPI_ReceiveData8(SPI3);  /// Data read from SPI serial data output after completion of Tx & while Rx is enabled
    return byte;

    GPIOA->ODR |=  (0x0001 << 4);           // PA_4 : CS = 1 - End Transmission

}

uint8_t gyro_send(uint8_t val)
{

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }

    SPI_SendData8(SPI3, val);              // Address/data sent to SPI serial data input while Tx is enabled

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { }

    return 0;

    //while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) { }
    //uint8_t byte = SPI_ReceiveData8(SPI3);  // Data read from SPI serial data output after completion of Tx & while Rx is enabled
    //return byte;
}
/*
void transmit_byte_spi3(uint8_t data)
{
    data &= 0x7F;
    SPI3->DR = data;
    GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be corrected

    SPI_SendData8(SPI3, data);

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) { } //need to be corrected
    GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
}

uint8_t read_byte_spi3(uint8_t read){
    read |= 0x80;
    SPI3->DR = read;
    GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { } //need to be corrected

    uint8_t byte = SPI_ReceiveData8(SPI3);

    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) { } //need to be corrected
    GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
return byte;
}


void transmit_string_spi3(const char* str, uint8_t lenght){
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be corrected
    GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    for (int n = 0; n < lenght; n++)
        {
            uint8_t data = str[n];
            SPI_SendData8(SPI3, data);
        }
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be corrected
    GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
}
*/

void setup_gyro()
{
    GPIOA->ODR &= ~(0x0001 << 4);           // PA_4 : CS = 0 - To start transmission
    gyro_send(CTRL_REG4 & 0x7F);        ///Select CTRL_REG4 by sending the address & (enable write operation)
    gyro_send(0x80);                        // BDU-1 (1: output registers not updated until MSB and LSB reading)
    gyro_send(CTRL_REG5 & 0x7F);        ///Select CTRL_REG5 by sending the address & (enable write operation)
    gyro_send(0xC0);                        // Reboot memory content & FIFO enable
    gyro_send(CTRL_REG1 & 0x7F);        ///Select CTRL_REG1 by sending the address & (enable write operation)
    gyro_send(0xDF);                        // Set power to normal mode and Enable x, y and z axis reading & Set Output Data Rate, Bandwidth
    GPIOA->ODR |=  (0x0001 << 4);        // PA_4 : CS = 1 - To end transmission
}

void gyro_test()
{
    gyro_read(WHO_AM_I);                    //To load the address of WHO_AM_I register
    uint8_t value = gyro_read(WHO_AM_I);    ///To read the value stored in the register

    if(value == 0xD3)                       //To verify if the value is read correctly
    {
        printf("\n Gyro is read correctly: %x  ", value); /// To print the reference value in hexadecimal
    }
    else
    {
        printf("\n Gyro is not read correctly ");
    }
}

void gyro_temp_val()
{
    gyro_read(OUT_TEMP);            //To load the address of output temperature register
    int temp_val = gyro_read(0xFF); ///After loading address, sending a random value 0xFF to get the output temperature stored in output register
                                    ///Successful read is observed only after two Tx & Rx cycles
    printf("| Temperature > %d \n", temp_val); //To print the 8-bit temperature data in decimal
}

void getGyroValues()
{
    uint16_t x, y, z;
    gyro_read(OUT_X_H | 0x40);    //To load the address OUT_X_H where the higher order 8-bits are stored
    x = ((gyro_read(0xFF))<<8);       ///To read the data from the register, a random value 0xFF is sent
    gyro_read(OUT_X_L | 0x40);    //To load the address OUT_X_L where the lower order 8-bits are stored
    x |= gyro_read(0xFF);             ///To read the data from the register, a random value 0xFF is sent

    gyro_read(OUT_Y_H | 0x40);      //Similar process is repeated for the y and z axes
    y = ((gyro_read(0xFF))<<8);
    gyro_read(OUT_Y_L | 0x40);
    y |= gyro_read(0xFF);

    gyro_read(OUT_Z_H | 0x40);
    z = ((gyro_read(0xFF))<<8);
    gyro_read(OUT_Z_L | 0x40);
    z |= (gyro_read(0xFF));

    signed int a = convert_to_decimal(x);
    signed int b = convert_to_decimal(y);
    signed int c = convert_to_decimal(z);

    printf("\n| X : Y : Z => %5d : %5d : %5d ", a, b, c);
}

/*
uint16_t getGyroValues(uint8_t axis)
{
    uint8_t cmd = (GYRO_READ_DATA | GYRO_MULTIPLE | axis);
    uint8_t result[2];  // [0] holds low byte, [1] holds high byte
    gyro_cs(CS_ACTIVE);
    gyro_read(cmd);    // Reads out 2 bytes from the gyro registers by applying the GYRO_MULTIPLE in the read command, and then transmitting as many dummy bytes as we want to read out.
    result[0] = gyro_read(0xFF);
    result[1] = gyro_read(0xFF));
    gyro_cs(CS_IDLE);
    return ((result[1] << 8) | result[0]);
}
*/

///The x, y & z outputs of Gyrometer are received in 2's complement form which are converted to decimal form using this function
void convert_to_decimal (uint16_t twos_complement)
{
    int signed_integer;
    if (twos_complement & 0b1000000000000000)
        {
            signed_integer = twos_complement | ~(0b0111111111111111);
        }
    else
        {
            signed_integer = (~(twos_complement) | 0b0000000000000001);
        }
    return signed_integer;
}

int main(void)
{
    init_usb_uart(9600);                  //Initialize UART
    init_spi3();                          ///Initialize SPI_3, which uses CLK: PB_3, MISO: PB_4, MOSI: PB_5, CS: PA_4

    setup_gyro();                         //Setup gyro to read the X Y Z angular orientation
    gyro_test();                          ///Test gyro if it reads WHO_AM_I register value(Performed without setup_gyro();)
    gyro_temp_val();                      //Test gyro if it reads temperature (Performed without setup_gyro();)
    while(1)
    {
        GPIOA->ODR &= ~(0x0001 << 4);
        GPIOA->ODR |=  (0x0001 << 4);       ///CS made 0 and 1 to initiate data read after reset
        getGyroValues();

    }
}
