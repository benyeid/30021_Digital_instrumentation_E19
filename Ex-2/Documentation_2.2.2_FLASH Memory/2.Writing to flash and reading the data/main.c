#include "stm32f30x_conf.h"
#include <stdio.h>
#include <string.h>
#include "flash.h"
#include "lcd.h"
#include "30021_io.h"

int main(void)
{
            init_usb_uart( 9600 );           /// Initialize USB serial emulation at 9600 baud
            uint32_t address = 0x0800F800;   /// Set address value to the last page of flash memory, to keep the code undisturbed

            uint16_t data[22];               /// data to edit the 1st 22 elements of Flash

            for ( int i = 0 ; i < 22 ; i++ ) data[i] = i;

            FLASH_Unlock();
            FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
            FLASH_ErasePage( address );
                    for ( int i = 0; i < 22; i++ )
                            {
                                FLASH_ProgramHalfWord(address + i * 2, data[i]); /// To write data to registers
                            }
            FLASH_Lock();

            uint16_t tempVal;
            for ( int i = 0 ; i < 22 ; i++ )              /// For reading the data from 1st 22 elements of the register
            {
                tempVal = *(uint16_t *)(address + i * 2); /// To store the value of address in tempval
                printf("%d\n", tempVal);                  /// To print the value stored in the register
            }
    while(1){}
}