#ifndef _GPIO_H
#define _GPIO_H
#include "stm32f30x_conf.h"

#define PIN_MODE_INPUT 0x0
#define PIN_MODE_OUTPUT 0x1
#define PIN_MODE_ALTERNATE 0x2
#define PIN_MODE_ANALOG 0x3
#define PIN_MODE_RESET PIN_MODE_INPUT

#define PIN_OTYPE_PP 0x0
#define PIN_OTYPE_OD 0x1
#define PIN_OTYPE_RESET PIN_OTYPE_PP

#define PIN_PUPD_NONE 0x0
#define PIN_PUPD_PU 0x1
#define PIN_PUPD_PD 0x2
#define PIN_PUPD_RESERVED 0x3

/// \brief Init given pin of given port.
/// \param GPIO_TypeDef *GPIO
/// \param uint8_t pin
/// \param  uint8_t mode
/// \param uint8_t pupd
/// \param int8_t otype
int initPin(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t mode, uint8_t pupd, uint8_t otype);

/// \brief Init given pin to alternating i/o
/// \param uint8_t pin
/// \param uint8_t AF
/// Depends on GPIO_PinAFConfig() in STM standard library and initPin defined above
int initPinAlternate(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t AF);

/// \brief Init port for RGB LED.
/// Operation: call initPin(GPIOB, color, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_OD);
/// where color has the following meaning:
/// - RED: 4
/// - GREEN: 7
// - BLUE: 9
int initLed(void);

/// \brief Set the LED to dedicated color.
/// RED is reached via GPIO B pin 4
/// GREEN is reached via GPIO C pin /
/// BLUE is reached via GPIO A pin 9
/// \param uint8_t color. It is masked as follows:
///    int red = (color & 1<<2)>>2;
///    int green = (color & 1<<1)>>1;
///    int blue = color & 1;
void setLed(uint8_t color);


int initJoystick(void);

/// \brief Read joystick position. Operation is as follow:
/// uint8_t up = (GPIOA->IDR & 0x10) >> 4;
///    uint8_t down = (GPIOB->IDR & 0x1);
///    uint8_t left = (GPIOC->IDR & 0x2) >> 1;
///    uint8_t right = (GPIOC->IDR & 0x1);
///    uint8_t center = (GPIOB->IDR & 0x20) >> 5;
/// \return joyAndHappiness |= up | down << 1 | left << 2 | right << 3 | center << 4;
uint8_t readJoystick(void);

#endif
