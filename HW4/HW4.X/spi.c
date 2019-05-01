//.c file for SPI1

#include "spi.h"
#include <xc.h>

void spi1_init(){
    SPI1CON = 0 ;               //reseting the spi module
    SPI1BUF;                    // clear the rx buffer by reading from it
    SPI1BRG = 0x2;            // baud rate to 8 MHz [SPI4BRG = (80000000/(2*desired))-1]
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1STATbits.SPIROV = 0;    // clear the overflow bit
    SPI1CONbits.MODE32 = 0;     // use 8 bit mode
    SPI1CONbits.MODE16 = 1; 
    SPI1CONbits.MSTEN = 1;      // master operation
    SPI1CONbits.ON = 1;         // turn on spi 1
    
    //LATBbits.LATB7 = 0;                   // enable the ram
    //spi1_io(0x01);                        // ram write status
    //spi1_io(0x41);                        // sequential mode (mode = 0b01), hold disabled (hold = 0)
    //LATBbits.LATB7 = 1;                   // finish the command
    
}


unsigned short spi1_io(unsigned short write){
    SPI1BUF = write ;
    while(!SPI1STATbits.SPIRBF) { ; }// wait to receive the byte
  return SPI1BUF;
}
