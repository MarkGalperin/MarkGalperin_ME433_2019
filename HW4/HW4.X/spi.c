//.c file for SPI1

#include "spi.h"
#include <xc.h>

void spi1_init(){
    SPI1BUF;                    // clear the rx buffer by reading from it
    SPI1BRG = 0x4;              // baud rate to 8 MHz [SPI4BRG = (80000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;    // clear the overflow bit
    SPI1CONbits.MODE32 = 0;     // use 8 bit mode
    SPI1CONbits.MODE16 = 0; 
    SPI1CONbits.MSTEN = 1;      // master operation
    SPI1CONbits.ON = 1;         // turn on spi 1
}


char spi1_io(char write){
    SPI1BUF = write ;
    return SPI1BUF ;
}
