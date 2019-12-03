#include "stm32f30x_conf.h"
#include "30021_io.h"
#include "accel.h"
#include "mag.h"
#include "spi.h"
#include <stdio.h>

int main(void)
{
    uint8_t CS;
    int16_t x,y,z;
    uint8_t id, status;
    init_usb_uart(9600);
    CS = 15;
    spi3_init(CS);

    if (CS == 15){
        accel_setup();
    }else if (CS == 4){
        mag_setup();
    }

    x = y = z = 0;
    while(1)
    {
        if (CS == 15){
            accel_whoami();
           //accel_getValues();
        }else if (CS == 4){
            mag_whoami();
           //mag_getValues();
        }
        for(int i = 0; i < 100; ++i);
    }
}
