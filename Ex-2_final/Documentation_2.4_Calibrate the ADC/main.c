#include "stm32f30x_conf.h"
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "lcd.h"
#include "30021_io.h"

#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))  //calibrated at 3.3v @ 30c

    void write_to_flash(float f1, float f2)
    {
            uint32_t tempfloat;
            uint32_t tempval;

            tempfloat = read_float_flash(PG31_BASE,0);
            init_page_flash(PG31_BASE);

                        FLASH_Unlock();
                        write_float_flash(PG31_BASE,0,f1);      ///  calibration factors stored in flash
                        write_float_flash(PG31_BASE,1,f2);      ///  calibration factors stored in flash
                        FLASH_Lock();

                tempfloat = read_float_flash(PG31_BASE,0);
                tempval = read_word_flash(PG31_BASE,0);
}
void ADC_setup_PA(void)
{
        RCC->CFGR2 &= ~RCC_CFGR2_ADCPRE12;          // Clear ADC12 prescaler bits
        RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV8;      // Set ADC12 prescaler to 8
        RCC->AHBENR |= RCC_AHBPeriph_ADC12;         // Enable clock for ADC12

        ADC1->CR = 0x00000000;                  // Clear CR register
        ADC1->CFGR &= 0xFDFFC007;               // Clear ADC1 config register
        ADC1->SQR1 &= ~ADC_SQR1_L;              // Clear regular sequence register 1

            //Verify ADVREG state (ADC voltage regulator)
            //1. Set ADVREGEN from '10' (disabled state) to '00' and then to '01' (enabled)
            //2. Wait 10uS (worst case) before performing calibration and/or
            ADC1->CR &= ~ADC_CR_ADVREGEN;
            ADC1->CR |= ADC_CR_ADVREGEN_0;
            //Wait for at least 10uS before continuing...
                for(uint32_t i = 0; i < 10000; i++);

            ADC1->CR &= ~ADC_CR_ADEN; //Make sure ADC is disabled
            ADC1->CR &= ~ADC_CR_ADCALDIF; //Use single ended calibration
            ADC1->CR |= ADC_CR_ADCAL; //Start calibration
                while((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL){} //Wait for calibration to finish
            //Wait > 4 ADC clock cycles after ADCAL bit is cleared
                for(uint32_t i = 0; i < 100; i++);

            //Enable ADC peripheral
            ADC1->CR |= ADC_CR_ADEN;
            //wait for ADC1 to be ready to start conversion
                while (!ADC1->ISR & ADC_ISR_ADRD){}
}

uint16_t ADC_measure_PA(uint8_t ch)
{
            ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_1Cycles5);

            ADC_StartConversion(ADC1); // Start ADC read
                while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read

            uint16_t x = ADC_GetConversionValue(ADC1); // Read the ADC value
            return x;
}

int main(void)
{
    float vdda;
    float factor1=1.003125, factor2=1.0025;
    char value1[31];
    char value2[31];
    char str[15];
    uint8_t fbuffer[512];

    ///Enabling clocks for Port: A
        RCC -> AHBENR |= RCC_AHBPeriph_GPIOA;

    /// Configured A0 and A1 to normal input mode
        GPIOA->MODER &= ~(0x00000003 << (0*2) | 0x00000003 << (1*2));
        GPIOA->MODER |=  (0x00000000 << (0*2) | 0x00000000 << (1*2));

        GPIOA->PUPDR &= ~(0x00000003 << (0*2) | 0x00000003 << (1*2));
        GPIOA->PUPDR |=  (0x00000002 << (0*2) | 0x00000002 << (1*2));

    init_usb_uart(9600);
    init_spi_lcd();
    write_to_flash(factor1, factor2);  ///  calibration factors stored in flash

    memset(fbuffer,0x00,512);
    lcd_push_buffer(fbuffer);

    ADC_setup_PA();

    ADC1_2->CCR |= ADC12_CCR_VREFEN;
    ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_181Cycles5);
    ADC_StartConversion(ADC1);       // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read

    float Vref_data = ADC_GetConversionValue(ADC1); // Read the ADC value
    vdda = 3.3 * VREFINT_CAL/Vref_data;

    while(1)
        {
/*
    ADC1: displayed value = 3.2 V; Multimeter value = 3.21 V ==> correction factor = 1.003125
    ADC2: displayed value = 3.2 V; Multimeter value = 3.208 V ==> correction factor = 1.0025
*/
            float cfact1 = read_float_flash(PG31_BASE, 0); /// To read the Calibration factor 1 from flash memory
            float cfact2 = read_float_flash(PG31_BASE, 1); /// To read the Calibration factor 2 from flash memory

            uint16_t Ch1 = ADC_measure_PA(ADC_Channel_1); ///Measure ADC1 count from the register
            uint16_t Ch2 = ADC_measure_PA(ADC_Channel_2); ///Measure ADC2 count from the register

            float Vc1 = (vdda/4095)*Ch1*cfact1;  ///Convert the read ADC1data to float
            float Vc2 = (vdda/4095)*Ch2*cfact2;  ///Convert the read ADC2 data to float

            snprintf(str,15,"Vdda=%.2f",vdda);
                printf(str); printf("\n");      ///stores and prints the value of Vdda = 3.32 V

            snprintf(value1,30,"%.2f,cfact1:%.6f",Vc1, cfact1);      ///To store ADC1 data to the string 'value1'
                snprintf(value2,30,"%.2f,cfact2:%.6f",Vc2, cfact2);  ///To store ADC2 data to the string 'value2'

           /* printf("ADC1 value=");printf(value1); printf("  ||||  ");
            printf("ADC2 value=");printf(value2); printf("\n");*/

            lcd_write_string("ADC1 value=",fbuffer,0,0);
            lcd_write_string(value1,fbuffer,0,1);                   ///To print ADC1 data to LCD
            lcd_write_string("ADC2 value=",fbuffer,0,2);
            lcd_write_string(value2,fbuffer,0,3);                   ///To print ADC1 data to LCD
            lcd_push_buffer(fbuffer);
        }
}
