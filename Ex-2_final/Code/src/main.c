#include "stm32f30x_conf.h"
#include "flash.h"
#include "lcd.h"
//#include "charset.h"
#include "30021_io.h"
/*int main(void)
{
    lcd_reset();
    init_spi_lcd();

    uint8_t fbuffer[512];

    memset(fbuffer,0xAA,512);

    while(1)
    {
        lcd_push_buffer(fbuffer);
    }
}
*/
int main(void)
{

    init_usb_uart( 9600 ); // Initialize USB serial emulation at 9600 baud
    int a=120;
    uint8_t fbuffer[512];
         // lcd_reset();
           init_spi_lcd();

           memset(fbuffer,0xAA,512);
    while(1)
        {

           lcd_push_buffer(fbuffer);
            printf("a = %d", a);

        }
}
