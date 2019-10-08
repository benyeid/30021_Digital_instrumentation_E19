#ifndef GROUP6_FUNCTIONS_H_INCLUDED
#define GROUP6_FUNCTIONS_H_INCLUDED

void ADC_setup_PA(void);

uint16_t ADC_measure_PA(uint8_t ch);

void timer16_clock_init();

void PORTC_outputs_config();

void GPIO_set_AF1_PA6();

void timer16_pwm_init();

int getCCR1ValueFromPercentage(double percent);

#endif /*! GROUP6_FUNCTIONS_H_INCLUDED */
