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

#include "stm32f30x_conf.h" // STM32 config
#include "30021_io.h" // Input/output library for this course

GPIO_InitTypeDef gpio;

typedef struct volatile _time{
uint32_t second;
uint32_t minute;
uint32_t hour;
} TIME;

int16_t power(int16_t a, int16_t exp) // need to change the type of the variables to avoid negative numbers
{
// calculates a^exp
    int16_t i, r = a;
    for (i = 1; i < exp; i++)// remove = character
        r *= a;
    return(r);
}
//-------------------------------------
//FIGYELEM! Az extra kártya neve mbed enabled!!

void initJoystick() {       //for Ex. 1.0

RCC -> AHBENR |= RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC;   ///Enabling clocks for Ports: A, B and C

///setting PC0 and PC1 to input (PC0 is connected to the right state of joystick, while PC1 - to the left)
GPIOC->MODER &=~ (0x00000003 << (0*2) | 0x00000003 << (1*2));                       ///Clearing mode register
GPIOC->MODER |=  (0x00000000 << (0*2) | 0x00000000 << (1*2));                       ///Setting PC0 and PC1 as inputs

GPIOC->PUPDR &=~ (0x00000003 << (0*2) | 0x00000003 << (1*2));                       ///Clearing Push/Pull register
GPIOC->PUPDR |=  (0x00000002 << (0*2) | 0x00000002 << (1*2));                       ///Setting as pull-down inputs

///setting PA4 to input
GPIOA->MODER &=~(0x00000003 << (4*2));
GPIOA->MODER |= (0x00000000 << (4*2));
GPIOA->PUPDR &=~(0x00000003 << (4*2));
GPIOA->PUPDR |= (0x00000002 << (4*2));

///setting PB0 and PB5 to input
GPIOB->MODER &=~(0x00000003 << (0*2) | 0x00000003 << (5*2));
GPIOB->MODER |= (0x00000000 << (0*2) | 0x00000000 << (5*2));

GPIOB->PUPDR &=~(0x00000003 << (0*2) | 0x00000003 << (5*2));
GPIOB->PUPDR |= (0x00000002 << (0*2) | 0x00000002 << (5*2));

}

/*
void initJoystick() {       //for Ex. 1.0

RCC -> AHBENR |= RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC;

//setting PC0 and PC1 to input
GPIOC->MODER &=~ (0x00000003 << (0*2) | 0x00000003 << (1*2));
GPIOC->MODER |=  (0x00000000 << (0*2) | 0x00000000 << (1*2));

GPIOC->PUPDR &=~ (0x00000003 << (0*2) | 0x00000003 << (1*2));
GPIOC->PUPDR |=  (0x00000002 << (0*2) | 0x00000002 << (1*2));

//setting PA4 to input
GPIOA->MODER &=~(0x00000003 << (4*2));
GPIOA->MODER |= (0x00000000 << (4*2));
GPIOA->PUPDR &=~(0x00000003 << (4*2));
GPIOA->PUPDR |= (0x00000002 << (4*2));

//setting PB0 and PB5 to input
GPIOB->MODER &=~(0x00000003 << (0*2) | 0x00000003 << (5*2));
GPIOB->MODER |= (0x00000000 << (0*2) | 0x00000000 << (5*2));

GPIOB->PUPDR &=~(0x00000003 << (0*2) | 0x00000003 << (5*2));
GPIOB->PUPDR |= (0x00000002 << (0*2) | 0x00000002 << (5*2));

}*/

Read_Joystick(int joystick_no)          //a function that converts the detected state of joystick to the required binary result
{
    int c,k;
    scanf("%d", &joystick_no);

    printf("The binary result is:\n", joystick_no);

    for (c = 7; c >= 0; c--)
    {
        k = joystick_no >> c;

        if (k & 1)
            printf("1");
        else
            printf("0");
    }
    printf("\n");
}

void initLed(){             //for Ex.1.1

RCC -> AHBENR |= RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC;

//setting PA9 to output
GPIOA->OSPEEDR &=~(0x00000003 << (9 * 2));
GPIOA->OSPEEDR |= (0x00000002 << (9 * 2)); //2MHz

GPIOA->OTYPER &=~(0x0001 << (9));
GPIOA->OTYPER |= (0x0001 << (9));//open-drain type

GPIOA->MODER &= ~(0x00000003 << (9 * 2)); // Clear mode register
GPIOA->MODER |= (0x00000001 << (9 * 2));

uint16_t val_a9 = GPIOA->IDR & (0b0001 >> 9);
GPIOA->ODR |= (0b0000 >> 9);        //set Pin PA9 to low

//setting PB4 to output
GPIOB->OSPEEDR &=~(0x00000003 << (4 * 2));
GPIOB->OSPEEDR |= (0x00000002 << (4 * 2)); //2MHz

GPIOB->OTYPER &=~(0x0001 << (4));
GPIOB->OTYPER |= (0x0001 << (4));//open-drain type

GPIOB->MODER &= ~(0x00000003 << (4 * 2)); // Clear mode register
GPIOB->MODER |= (0x00000001 << (4 * 2));

uint16_t val_b4 = GPIOB->IDR & (0x0001 << 0);
GPIOB->ODR |= (0b0000 >> 4);            //set to low

//setting PC7 to output
GPIOC->OSPEEDR &=~(0x00000003 << (7 * 2));
GPIOC->OSPEEDR |= (0x00000002 << (7 * 2)); //2MHz

GPIOC->OTYPER &=~(0x0001 << (7));
GPIOC->OTYPER |= (0x0001 << (7));//open-drain type

GPIOC->MODER &= ~(0x00000003 << (7 * 2)); // Clear mode register
GPIOC->MODER |= (0x00000001 << (7 * 2));

uint16_t val_c7 = GPIOC->IDR & (0x0001 << 0);
GPIOC->ODR |= (0b0000 >> 7);                //set to low
}

void init_interrupt()
{
    uint32_t prio=1;

    NVIC_SetPriority(EXTI0_IRQn,prio); // set interrupt prio
    NVIC_EnableIRQ(EXTI0_IRQn); //Enable ITs


    SYSCFG->EXTICR[0] &= 0x0001; /// Select PA0 to make IRQ
    EXTI->RTSR |= 0x001; ///Rising trigger
    EXTI->IMR |= 0x001; /// IT request is not masked
}

void initTimer()
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 = 0x00;
    TIM2->ARR = 630u;
    TIM2->PSC = 0x00;

    TIM2->DIER |= 0x0001;
    NVIC_SetPriority(TIM2_IRQn, 1); // Set interrupt priority
    NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt
}
Set_Led(int joystick){
    switch (joystick)
    {   case 1:                                      //1 is related to joystick number = 1, so PA4 has been changed
            GPIO_WriteBit(GPIOA, 0x0200, 0);
            GPIO_WriteBit(GPIOC, 0x0080, 0);                    //configuration resulting in white
            GPIO_WriteBit(GPIOB, 0x0010, 0);
        break;
        case 2:                                    //2 is related to joystick number = 2, so PB0 has been changed
            GPIO_WriteBit(GPIOA, 0x0200, 1);
            GPIO_WriteBit(GPIOC, 0x0080, 0);                    //configuration resulting in green
            GPIO_WriteBit(GPIOB, 0x0010, 1);
        break;
        case 4:
            GPIO_WriteBit(GPIOA, 0x0200, 0);
            GPIO_WriteBit(GPIOC, 0x0080, 1);                    //configuration resulting in pink
            GPIO_WriteBit(GPIOB, 0x0010, 0);
        break;
        case 8:
            GPIO_WriteBit(GPIOA, 0x0200, 1);
            GPIO_WriteBit(GPIOC, 0x0080, 1);                    //configuration resulting in red
            GPIO_WriteBit(GPIOB, 0x0010, 0);
        break;
        case 16:
            GPIO_WriteBit(GPIOA, 0x0200, 0);
            GPIO_WriteBit(GPIOC, 0x0080, 1);                    //configuration resulting in blue
            GPIO_WriteBit(GPIOB, 0x0010, 1);
        break;
    }

}


void EXTIO_IRQHandler(void){

    printf("The interruption has been performed. After a while, Joystick and LED will be active again.");

    while (1)
    {
        if (GPIO_ReadInputDataBit(GPIOB, 0x0020)!=0u){      //if PB5 state changed - center: the result should be 0001 0000
            break;
        }

    }

    EXTI->PR |= EXTI_PR_PR0;    //clearing the interrupt pending bit
    return 0;
 }



int main(void)
{
    init_usb_uart(9600);
    printf("HI \n");

    initJoystick();
    initLed();
    init_interrupt();

    initTimer();

    GPIO_WriteBit(GPIOA, 0x0200, 1);
    GPIO_WriteBit(GPIOC, 0x0080, 1);        //our RGB LED is turned off
    GPIO_WriteBit(GPIOB, 0x0010, 1);

    int joystick_number;


    while(1)
    {
        /*if(!((Read_Pin_State(GPIOA)&(0x0200))|(Read_Pin_State(GPIOB)&(0x0020))|(Read_Pin_State(GPIOB)&(0x0000))|(Read_Pin_State(GPIOC)&(0x0000))|(Read_Pin_State(GPIOC)&(0x0001))))
        {
            Read_Joystick();
            printf(joystick_no);
        }*/

        //------------------------------------------------------Ex.1.1---------------------------------------------------------------------------------------
        if (GPIO_ReadInputDataBit(GPIOC, 0x0002)!=0){            //if PC1 state changed - left direction: result should be 0000 0100
            joystick_number = 4;
            }
        else if (GPIO_ReadInputDataBit(GPIOB, 0x0020)!=0){      //if PB5 state changed - center: the result should be 0001 0000
            joystick_number = 16;
            EXTIO_IRQHandler();
            }
        else if (GPIO_ReadInputDataBit(GPIOB, 0x0001)!=0){      //if PB0 state changed - down: the result should be 0000 0010
            joystick_number = 2;

            }
        else if (GPIO_ReadInputDataBit(GPIOC, 0x0001)!=0){      //if PC0 state changed - right: 0000 1000
            joystick_number = 8;

            }
        else if (GPIO_ReadInputDataBit(GPIOA, 0x0010)!=0){      //if PA4 state changed - up: 0000 0001
            joystick_number = 1;
            }

        Read_Joystick(joystick_number);
 //---------------------------------------------------------Ex.1.2-----------------------------------------------------------------------------------
        Set_Led(joystick_number);








        //the only problem is that the results are now printed all the time, we should add an interrupt I guess
//---------------------------------------------------------------------------------------------------------------------------------------------------

 //---------------------------------------------------------Ex.1.4-----------------------------------------------------------------------------------




    }

    //-----------------For Assignment 0
    /*int16_t a;
    init_usb_uart( 9600 ); // Initialize USB serial at 9600 baud
    printf("\n\n x x^2 x^3 x^4\n");
    for (a = 0; a < 10; a++)
        printf("%8d%8d%8d%8d\n",a, power(a, 2), power(a, 3), power(a, 4));
    while(1) {}*/
}
