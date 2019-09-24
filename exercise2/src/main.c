#include "stm32f30x_conf.h"
#include "flash.h"
#include "lcd.h"
#include "charset.h"

int main(void)
{
    lcd_reset();
    init_spi_lcd();

    uint8_t fbuffer[512];

    memset(fbuffer,0xAA,512);

    lcd_push_buffer(fbuffer);


    while(1)
    {

    }
}
