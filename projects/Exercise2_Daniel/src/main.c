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

#include "group6_functions.h"

#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))  //calibrated at 3.3v @ 30c

int main(void)
{
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

    init_usb_uart(9600);

    timer16_clock_init();
    GPIO_set_AF1_PA6();
    timer16_pwm_init();


  while(1)
  {
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
        }
    }
}

