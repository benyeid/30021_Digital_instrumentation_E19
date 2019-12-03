#include "stm32f30x_conf.h"
#include "gpio.h"

timer16_clock_init();
timer17_clock_init();
GPIO_set_AF1_PA7();
GPIO_set_AF1_PA6();
timer16_pwm_init();
timer17_pwm_init();

