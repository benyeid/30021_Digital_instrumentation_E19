#include "stm32f30x_conf.h"
#include "30021_io.h"
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "lcd.h"
#include <time.h>
#include <inttypes.h>


void init_spi3() {
    // Enable Clocks
    RCC->AHBENR  |= 0x00020000 | 0x00040000| 0x00080000;    // Enable Clock for GPIO Banks A, B & C
    RCC->APB1ENR |= 0x00008000;                 // Enable Clock for SPI3

    // Connect pins to SPI3
    GPIOC->AFR[10 >> 0x03] &= ~(0x0000000F << ((10 & 0x00000007) * 4)); // Clear alternate function for PC10
    GPIOC->AFR[10 >> 0x03] |=  (0x00000006 << ((10 & 0x00000007) * 4)); // Set alternate 6 function for PC10 - SCLK
    GPIOC->AFR[12 >> 0x03] &= ~(0x0000000F << ((12 & 0x00000007) * 4)); // Clear alternate function for PC12
    GPIOC->AFR[12 >> 0x03] |=  (0x00000006 << ((12 & 0x00000007) * 4)); // Set alternate 6 function for PC12 - MOSI
    //GPIOB->AFR[4 >> 0x03] &= ~(0x0000000F << ((4 & 0x00000007) * 4)); // Clear alternate function for PB4
    //GPIOB->AFR[4 >> 0x03] |=  (0x00000006 << ((4 & 0x00000007) * 4)); // Set alternate 6 function for PB4 - MiSo


    /// Configure pins PC10 (SCLK) and PC12(MOSI) for 10 MHz alternate function
    GPIOC->OSPEEDR &= ~(0x00000003 << (10 * 2) | 0x00000003 << (12 * 2));    // Clear speed register
    GPIOC->OSPEEDR |=  (0x00000001 << (10 * 2) | 0x00000001 << (12 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOC->OTYPER  &= ~(0x0001     << (10)     | 0x0001     << (12));        // Clear output type register
    GPIOC->OTYPER  |=  (0x0000     << (10)     | 0x0000     << (12));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOC->MODER   &= ~(0x00000003 << (10 * 2) | 0x00000003 << (12 * 2));    // Clear mode register
    GPIOC->MODER   |=  (0x00000002 << (10 * 2) | 0x00000002 << (12 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOC->PUPDR   &= ~(0x00000003 << (10 * 2) | 0x00000003 << (12 * 2));    // Clear push/pull register
    GPIOC->PUPDR   |=  (0x00000000 << (10 * 2) | 0x00000000 << (12 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    /// Configurate PC11(MISO) for 10MHz alternate function
    GPIOC->OSPEEDR &= ~(0x00000003 << (11 * 2));    // Clear speed register
    GPIOC->OSPEEDR |=  (0x00000001 << (11 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOC->OTYPER  &= ~(0x0001     << (11));        // Clear output type register
    GPIOC->OTYPER  |=  (0x0000     << (11));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOC->MODER   &= ~(0x00000003 << (11 * 2));    // Clear mode register
    GPIOC->MODER   |=  (0x00000002 << (11 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOC->PUPDR   &= ~(0x00000003 << (11 * 2));    // Clear push/pull register
    GPIOC->PUPDR   |=  (0x00000000 << (11 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    ///setting PA15 (the CS)
    GPIOA->OSPEEDR &= ~(0x00000003 << (15 * 2));
    GPIOA->OSPEEDR |=  (0x00000001 << (15 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOA->OTYPER  &= ~(0x0001     << (15));        // Clear output type register
    GPIOA->OTYPER  |=  (0x0000     << (15));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOA->MODER   &= ~(0x00000003 << (15 * 2));    // Clear mode register
    GPIOA->MODER   |=  (0x00000001 << (15 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOA->PUPDR   &= ~(0x00000003 << (15 * 2));    // Clear push/pull register
    GPIOA->PUPDR   |=  (0x00000000 << (15 * 2));
    //uint16_t val = GPIOA->IDR & (0x0001 << 0);
    GPIOA->ODR |=  (0x0001 << 4); // CS = 1

    // Configure SPI3
    SPI3->CR1 &= 0x3040; // Clear CR1 Register
    SPI3->CR1 |= 0x0000; // Configure direction (0x0000 - 2 Lines Full Duplex, 0x0400 - 2 Lines RX Only, 0x8000 - 1 Line RX, 0xC000 - 1 Line TX)
    SPI3->CR1 |= 0x0104; // Configure mode (0x0000 - Slave, 0x0104 - Master)
    SPI3->CR1 |= 0x0010;   // clk = fclk/8
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

}


void transmit_byte_spi3() {
    GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be corrected
    assert_param(IS_SPI_ALL_PERIPH_EXT(SPIx));
    SPI3->DR = 'a';   ///Modified
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be corrected
    GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
}

void read_byte_spi3(){
    GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { } //need to be corrected
    //SPI_I2S_SendData16(SPI_TypeDef* SPIx, uint16_t Data)
    SPI_ReceiveData8(SPI3);
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { } //need to be corrected
    GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
}

void write_spi(){

}

/*
void transmit_setup(uint32_t baud){

RCC->AHBENR |= RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOA;
RCC->APB2ENR |= 0x4000;

///Connecting pins to USART
GPIOC->AFR[4 >> 0x03] &= ~(0x0000000F << ((4 & 0x00000007) * 4));       /// Clear alternate function for PC4
GPIOC->AFR[4 >> 0x03] |=  (0x00000007 << ((4 & 0x00000007) * 4));       /// Set alternate 7 function for PC4
GPIOC->AFR[5 >> 0x03] &= ~(0x0000000F << ((5 & 0x00000007) * 4));       /// Clear alternate function for PC5
GPIOC->AFR[5 >> 0x03] |=  (0x00000007 << ((5 & 0x00000007) * 4));       /// Set alternate 7 function for PC5
GPIOA->AFR[12 >> 0x03] &= ~(0x0000000F << ((12 & 0x00000007) * 4));     /// Clear alternate function for PA12
GPIOA->AFR[12 >> 0x03] |=  (0x00000007 << ((12 & 0x00000007) * 4));     /// Set alternate 7 function for PA12

///PC4 and PC5
GPIOC->OSPEEDR &= ~(0x00000003 << (4 * 2) | 0x00000003 << (5 * 2));     /// Clear speed register
GPIOC->OSPEEDR |=  (0x00000001 << (4 * 2) | 0x00000001 << (5 * 2));     /// Set speed register for 10 MHz
GPIOC->OTYPER  &= ~(0x0001     << (4)     | 0x0001     << (5));         /// Clear output type register
GPIOC->OTYPER  |=  (0x0000     << (4)     | 0x0000     << (5));         /// Set output type register as Push/Pull
GPIOC->MODER   &= ~(0x00000003 << (4 * 2) | 0x00000003 << (5 * 2));     /// Clear mode register
GPIOC->MODER   |=  (0x00000002 << (4 * 2) | 0x00000002 << (5 * 2));     /// Set mode register for Alternate Function
GPIOC->PUPDR   &= ~(0x00000003 << (4 * 2) | 0x00000003 << (5 * 2));     /// Clear push/pull register
GPIOC->PUPDR   |=  (0x00000001 << (4 * 2) | 0x00000001 << (5 * 2));     /// Set Push/Pull register


 ///Configure USART1
USART1->CR1 &=~0x00000001;                                         /// Disable USART1
USART1->CR2 &=~0x00003000;                                         /// Clear CR2 Configuration
USART1->CR2 |= 0x00000000;                                         /// Set 1 stop bits
USART1->CR1 &=~(0x00001000|0x00000400|0x00000200|0x00000008|0x00000004); /// Clear CR1 Configuration
USART1->CR1 |= 0x00000000;                                         /// Set word length to 8 bits
USART1->CR1 |= 0x00000000;                                         /// Set parity bits to none
USART1->CR1 |= USART_CR1_RE|0x00000008;                              /// Set mode to RX and TX
USART1->CR3 &=~(0x00000100|0x00000200);                            /// Clear CR3 Configuration
USART1->CR3 |= 0x00000000;                                         /// Set hardware flow control to none

    uint32_t divider = 0, apbclock = 0, tmpreg = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus); // Get USART2 Clock frequency
    apbclock = RCC_ClocksStatus.USART1CLK_Frequency;

    if ((USART1->CR1 & 0x00008000) != 0) {
      // (divider * 10) computing in case Oversampling mode is 8 Samples
      divider = (2 * apbclock) / baud;
      tmpreg  = (2 * apbclock) % baud;
    } else {
      // (divider * 10) computing in case Oversampling mode is 16 Samples
      divider = apbclock / baud;
      tmpreg  = apbclock % baud;
    }
    if (tmpreg >=  baud/2) {
        divider++;
    }
    if ((USART1->CR1 & 0x00008000) != 0) {
        // get the LSB of divider and shift it to the right by 1 bit
        tmpreg = (divider & (uint16_t)0x000F) >> 1;
        // update the divider value
        divider = (divider & (uint16_t)0xFFF0) | tmpreg;
    }
    USART1->BRR = (uint16_t)divider; // Configure baud rate
    USART1->CR1 |= 0x00000001; // Enable USART2
}

void uart_put_char(uint8_t c) {
    USART_SendData(USART1, (uint8_t)c);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}
}

int uart_put_string(const char *str, unsigned int lenght){
    for (int n = 0; n < lenght; n++) {
        if (str[n] == '\n') {
            uart_put_char('\r');
            }
        uart_put_char(str[n] & (uint16_t)0x01FF);
    }
    return lenght;
}
*/
int main(void)
{
    //uint8_t a=8;
    init_spi3();


    while(1)
    {
        for(int i=0; i<10000; i++);
        transmit_byte_spi3();
       // for(int i=0; i<10000; i++);
        //read_byte_spi3();
    }
}



