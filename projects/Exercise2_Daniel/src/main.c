/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/

#include "stm32f30x_conf.h"
#include "30021_io.h"
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "lcd.h"
#include <time.h>
#include <inttypes.h>

//#include "group6_functions.h"

#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))  //calibrated at 3.3v @ 30c

  //--------------------------------------------Exercise 5 functions----------------------------------------------------

void init_spi3() {
    // Enable Clocks

    RCC->AHBENR  |= 0x00020000 | 0x00040000;    // Enable Clock for GPIO Banks A and B
    RCC->APB1ENR |= 0x00008000;                 // Enable Clock for SPI3

    // Connect pins to SPI3
    GPIOB->AFR[3 >> 0x03] &= ~(0x0000000F << ((3 & 0x00000007) * 4)); // Clear alternate function for PB3
    GPIOB->AFR[3 >> 0x03] |=  (0x00000006 << ((3 & 0x00000007) * 4)); // Set alternate 6 function for PB3 - SCLK
    GPIOB->AFR[5 >> 0x03] &= ~(0x0000000F << ((5 & 0x00000007) * 4)); // Clear alternate function for PB5
    GPIOB->AFR[5 >> 0x03] |=  (0x00000006 << ((5 & 0x00000007) * 4)); // Set alternate 6 function for PB5 - MOSI
    //GPIOB->AFR[4 >> 0x03] &= ~(0x0000000F << ((4 & 0x00000007) * 4)); // Clear alternate function for PB4
    //GPIOB->AFR[4 >> 0x03] |=  (0x00000006 << ((4 & 0x00000007) * 4)); // Set alternate 6 function for PB4 - MiSo


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


void transmit_byte_spi3(uint8_t data) {
    GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be corrected
    //while (!(SPI1->SR & SPI_SR_TXE));
    SPI_SendData8(SPI3, data);
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be corrected
    GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
}

uint8_t read_byte_spi3(){
    GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { } //need to be corrected
    uint8_t byte = SPI_ReceiveData8(SPI3);
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { } //need to be corrected
    GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
return byte;
}

void transmit_string_spi3(const char* str, uint8_t lenght){
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be corrected
    GPIOA->ODR &= ~(0x0001 << 4);           // CS = 0 - Start Transmission
    for (int n = 0; n < lenght; n++){
        uint8_t data = str[n];
        SPI_SendData8(SPI3, data);
    }
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { } //need to be corrected
    GPIOA->ODR |=  (0x0001 << 4);           // CS = 1 - End Transmission
}

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


int main(void)
{
	init_spi3();
    init_usb_uart(9600);
    const char text[1] = "a";
	
	/*
    int POSITION[4] = {0b0101, 0b1001,0b1010, 0b0110};

    PORTC_outputs_config();

    int position = 0;
    float vdda;
    char value1[31];
    char value2[31];
    uint8_t fbuffer[512];

    ///Enabling clocks for Port: A
    RCC -> AHBENR |= RCC_AHBPeriph_GPIOA;

    /// Configured A0 and A1 to normal input mode
    GPIOA->MODER &= ~(0x00000003 << (0*2) | 0x00000003 << (1*2));
    GPIOA->MODER |=  (0x00000000 << (0*2) | 0x00000000 << (1*2));

    GPIOA->PUPDR &= ~(0x00000003 << (0*2) | 0x00000003 << (1*2));
    GPIOA->PUPDR |=  (0x00000002 << (0*2) | 0x00000002 << (1*2));

    uint8_t actual_bit = 0b0000;

    init_spi_lcd();
    memset(fbuffer,0x00,512);
    lcd_push_buffer(fbuffer);

    ADC_setup_PA();

    ADC1_2->CCR |= ADC12_CCR_VREFEN;
    ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_181Cycles5);
    ADC_StartConversion(ADC1);       // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
    uint16_t Vref_data = ADC_GetConversionValue(ADC1); // Read the ADC value
    vdda = 3.3 * VREFINT_CAL/Vref_data;

    //init_usb_uart(9600);

    timer16_clock_init();
    GPIO_set_AF1_PA6();
    timer16_pwm_init();*/


  while(1)
  {
	  
	  transmit_byte_spi3('a');
        //transmit_string_spi3(text, 1);

        for(int i = 0; i < 1000; i++);

        uint8_t byte = read_byte_spi3();
        printf('%d', byte);
	  
	  /*
        uint16_t Ch1 = ADC_measure_PA(ADC_Channel_1);
        float Vc1 = (vdda/4095)*Ch1;

        int dutyCykle = (78 * Vc1)/1;

        printf("%2d", dutyCykle);
        printf("\n");

        TIM16->CCR1 = dutyCykle;
        snprintf(value1,25,"%.2f",Vc1);

        lcd_write_string("ADC1 value=",fbuffer,0,0);
        lcd_write_string(value1,fbuffer,11,1);

        lcd_push_buffer(fbuffer);// printf("AAA");


  //--------------------------------------------Exercise 3.3.----------------------------------------------------
/*
        position++;
        actual_bit = POSITION[position&3];

        switch(position){
        case 1:
            ///0101
            GPIO_WriteBit(GPIOC, 0x0F, 0);
            GPIO_WriteBit(GPIOC, 0x08, 0);

            for(int i=0; i<=100000; i++);
            break;
        case 2:
            ///1001
            GPIO_WriteBit(GPIOC, 0x06, 0);
            GPIO_WriteBit(GPIOC, 0x0F, 0);

            for(int i=0; i<=100000; i++);
            break;
        case 3:
            GPIOC->ODR |= 0b00100001; //(0x0001 << 0) | (0x0001 << 5);
            ///1010
            GPIO_WriteBit(GPIOC, 0x06, 0);
            GPIO_WriteBit(GPIOC, 0x0E, 0);

            for(int i=0; i<=100000; i++);
            break;
        case 4:
            ///0110
            GPIO_WriteBit(GPIOC, 0x08, 0);
            GPIO_WriteBit(GPIOC, 0x0E, 0);

            for(int i=0; i<=100000; i++);
            position = 0;
            break;
        }*/
  //-------------------------------------------- end of Exercise 3.3.----------------------------------------------------
    }
}

