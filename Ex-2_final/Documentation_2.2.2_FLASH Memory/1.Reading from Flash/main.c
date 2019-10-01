#include "stm32f30x_conf.h"
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "lcd.h"

int main(void)
{
            init_usb_uart( 9600 );           /// Initialize USB serial emulation at 9600 baud
            uint32_t address = 0x0800F800;   /// Set address value to the last page of flash memory, 
			                                 /// to keep the code undisturbed
            uint16_t tempVal;
            for ( int i = 0 ; i < 10 ; i++ ) /// For reading the data from 1st 10 elements of the register
            {
                tempVal = *(uint16_t *)(address + i * 2); // Read Command
                printf("%d\n", tempVal);     ///To print the value stored in the register
            }
    while(1){}

}
