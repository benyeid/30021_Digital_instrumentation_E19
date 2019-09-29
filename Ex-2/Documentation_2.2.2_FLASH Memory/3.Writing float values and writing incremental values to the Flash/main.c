#include "stm32f30x_conf.h"
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "lcd.h"
#include "30021_io.h"

int main(void)
{
            init_usb_uart( 9600 );           /// Initialize USB serial emulation at 9600 baud
            uint32_t tempfloat;
            uint32_t tempval;

            tempfloat = read_float_flash(PG31_BASE,0);
            init_page_flash(PG31_BASE);

            tempfloat = read_float_flash(PG31_BASE,0);
            tempval = read_word_flash(PG31_BASE,0);
            if(tempval!=(uint32_t)0x01010101)     ///initialize a temporary variable with an incremental value
                {
                    init_page_flash(PG31_BASE);

                    for (int b=0; b<512 ;b++)     ///To fill the last page of the flash memory using incremental values
                        {
                            FLASH_Unlock();
                            write_word_flash(PG31_BASE,b,(uint32_t)0x01010101*(b));
                            FLASH_Lock();
                        }
                }
                tempval = read_hword_flash(PG31_BASE,0);
    while(1)
        {}
}
