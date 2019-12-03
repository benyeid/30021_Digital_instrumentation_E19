#include "stm32f30x_conf.h"
#include "gpio.h"

timer16_clock_init(){
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN | RCC_APB2Periph_TIM16;

    TIM16->PSC = 24;
    TIM16->ARR = 255;
    TIM16->DIER |= 0x0000;
    TIM16->CR1 = TIM_CR1_CEN;
	 //TIM16->CR2 &= ~TIM_CR2_TI1S;
}

timer17_clock_init(){
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN | RCC_APB2Periph_TIM17;

    TIM17->PSC = 24;
    TIM17->ARR = 255;
    TIM17->DIER |= 0x0000;
    TIM17->CR1 = TIM_CR1_CEN;
    //TIM16->CR2 &= ~TIM_CR2_TI1S;
}

GPIO_set_AF1_PA7(){
    RCC->AHBENR |= RCC_AHBPeriph_GPIOA;
    GPIOA->MODER &= ~(GPIO_MODER_MODER7);
    GPIOA->MODER |= GPIO_MODER_MODER7_1;
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR7; //reset
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR7_0;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);
}

GPIO_set_AF1_PA6(){
///alternate mode (PA6 connected to AF1; CH1)
//Set alternate function 1 for PA6 to map PA6 and TIM16_CH1
    RCC->AHBENR |= RCC_AHBPeriph_GPIOA; // Enable clock for GPIO Port A
 //Set mode register for A, mask PA6 (AlternateFunction)
 //Set mode register(0x00-Input,0x01-Output,0x02-AlternateFunction,0x03-Analog in/out)
    GPIOA->MODER &= ~(GPIO_MODER_MODER6);
    GPIOA->MODER |= GPIO_MODER_MODER6_1;
 //Set push/pull register(_0x00-No pull_,0x01-Pull-up,0x02-Pull-down)
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR6);
 //set speed register(0x01-10MHz, 0x02-2MHz,_0x03-50MHz_)
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR6; //reset
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1 | GPIO_OSPEEDER_OSPEEDR6_0;
 //Set alternate function register (GPIOx_AFRH) for PA6
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
}

timer16_pwm_init()
{
    TIM16->CCER &=~ TIM_CCER_CC1P &~ TIM_CCER_CC1E;
    TIM16->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1E;
//Setup the TIMx_CCMR1 register:
    TIM16->CCMR1 &= ~TIM_CCMR1_CC1S & ~TIM_CCMR1_OC1PE & ~TIM_CCMR1_OC1M;
    TIM16->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM16->CCMR1 |= TIM_CCMR1_OC1M_3|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_0;
 //Setup the TIM16 break/dead time register (TIMx_BDTR) to enable OCx output
    TIM16->BDTR &= ~TIM_BDTR_MOE & ~TIM_BDTR_OSSI & ~TIM_BDTR_OSSR;
    TIM16->BDTR |= TIM_BDTR_MOE;
 //Set duty cycle to 50% by writing the CCRx register
    TIM16->CCR1 = 128; //78; //for 1V //128 for 50%
 }

timer17_pwm_init(){
    TIM17->CCER &=~ TIM_CCER_CC1P &~ TIM_CCER_CC1E;
    TIM17->CCER |= TIM_CCER_CC1P | TIM_CCER_CC1E;
//Setup the TIMx_CCMR1 register:
    TIM17->CCMR1 &= ~TIM_CCMR1_CC1S & ~TIM_CCMR1_OC1PE & ~TIM_CCMR1_OC1M;
    TIM17->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM17->CCMR1 |= TIM_CCMR1_OC1M_3|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_0;
 //Setup the TIM16 break/dead time register (TIMx_BDTR) to enable OCx output
    TIM17->BDTR &= ~TIM_BDTR_MOE & ~TIM_BDTR_OSSI & ~TIM_BDTR_OSSR;
    TIM17->BDTR |= TIM_BDTR_MOE;
 //Set duty cycle to 50% by writing the CCRx register
    TIM17->CCR1 = 128; //78; //for 1V //128 for 50%
}

