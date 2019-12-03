#include <stdio.h>

#include "stm32f30x_conf.h"
#include "30021_io.h"
#include "gyro.h"
#include "accel.h"
#include "mag.h"
#include "spi.h"
#include "timers.h"
#include "controllers.h"

#define Gyro 4
#define Acc 15

int16_t accel_average(){
    int16_t data = 0;

    for(uint8_t i=0; i<20; i++){
        data += accel_getValues();
    }
    int16_t final_data = data/20;

    printf("Averaged read data = %d", final_data);

    return final_data;
}

int main(void)
{
    uint8_t count = 0;
    uint8_t CS, id, status;

    init_usb_uart(9600);
    CS = Acc;
    spi3_init(CS);

    if (CS == Acc)
    {
        accel_setup();
    }
    else if (CS == Gyro)
    {
        gyro_setup();

        while (SPI_GetReceptionFIFOStatus(SPI3) != SPI_ReceptionFIFOStatus_Empty && count < 4)
        {
            SPI3->DR;
            count++;
        }
    }

    timer16_clock_init();
    timer17_clock_init();

    GPIO_set_AF1_PA6();
	GPIO_set_AF1_PA7();

    timer16_pwm_init();
	timer17_pwm_init();

    while(1)
    {
        if (CS == Acc)
        {
            //accel_test();
            accel_getValues();
            balance_weight();
        }
        else if (CS == Gyro)
        {
            //gyro_test();
            gyro_getValues();
        }
    }
}
