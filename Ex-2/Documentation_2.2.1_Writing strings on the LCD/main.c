#include "stm32f30x_conf.h"
#include <stdio.h>
#include <string.h>
#include "lcd.h"

int main(void)
{
        uint8_t a = 121;
        init_spi_lcd();                               ///to setup SPI and configure the LCD
        char fbuffer[512];
        memset(fbuffer,0x20,512);                     ///To load the local buffer with desired data in ASCII

        lcd_write_string("Hi..", fbuffer, 0, 0);       ///To load the bytes of local buffer with required string 
        lcd_write_string("I'm Omega..", fbuffer, 21, 0);
        lcd_write_string("How are you???", fbuffer, 0, 1);
        lcd_write_string("Lokesh, Asia & Daniel",fbuffer, 0, 2);
        lcd_write_string("God Dag!!", fbuffer, 35, 3);

        lcd_push_buffer(fbuffer);                     ///To push the local buffer value to frame buffer of the LCD

    while(1)
        {
        }
}
