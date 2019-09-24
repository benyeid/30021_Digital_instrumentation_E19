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
#include "stm32f30x_conf.h"
#include "30021_io.h" // Input/output library for this course

#include "flash.h"
#include "lcd.h"
//#include "charset.h"

int main(void)
{
    init_usb_uart( 9600 ); // Initialize USB serial emulation at 9600 baud
    printf("Hello World!\n"); // Show the world you are alive!

    int a=120;
    uint8_t fbuffer[512];

    // lcd_reset();

    init_spi_lcd();
    memset(fbuffer,0xAA,512);
    while(1)
    {
        lcd_push_buffer(fbuffer);
        printf("a = %d ", a);
    }


}
