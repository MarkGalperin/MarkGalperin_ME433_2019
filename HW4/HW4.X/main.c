#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "spi.h"
#include <math.h>

// DEVCFG0 
#pragma config DEBUG = OFF // no debugging 
#pragma config JTAGEN = OFF // no jtag 
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1 
#pragma config PWP = OFF // no write protect 
#pragma config BWP = OFF // no boot write protect 
#pragma config CP = OFF // no code protect

// DEVCFG1 
#pragma config FNOSC = PRIPLL // use primary oscillator with pll 
#pragma config FSOSCEN = OFF // turn off secondary oscillator 
#pragma config IESO = OFF // no switching clocks 
#pragma config POSCMOD = HS // high speed crystal mode 
#pragma config OSCIOFNC = OFF // disable secondary osc 
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock 
#pragma config FCKSM = CSDCMD // do not enable clock switch 
#pragma config WDTPS = PS1048576 // use slowest wdt 
#pragma config WINDIS = OFF // wdt no window mode 
#pragma config FWDTEN = OFF // wdt disabled 
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal 
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz 
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV 
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz 
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB 
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3 
#pragma config USERID = 0 // some 16bit userid, doesn't matter what 
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations 
#pragma config IOL1WAY = OFF // allow multiple reconfigurations 
#pragma config FUSBIDIO = ON // USB pins controlled by USB module 
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATBbits.LATB7 //B7 is the chip select pin

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    //Initialize pins...
    //I'm setting B13 and B7 as SDO1 and SS1, respectively. B8 is SDI1 B14 is SCK1
    //TRISBbits.TRISB13 = 0 ; //B13 set to output
    TRISBbits.TRISB7 = 0 ;  //B7 set to output 
    //TRISBbits.TRISB8 = 0 ;  //B8 set to input
    
    
    RPB13Rbits.RPB13R = 0b0011 ;    //B13 mapped to SDO1
    // RPB7Rbits.RPB7R = 0b0011 ;   //B7 mapped to SS1 (CHANGE TO JUST LAT??)
    //SDI1Rbits.SDI1R = 0b0100 ;      //SDI1 mapped to RB8
    
    CS =1 ;
    
    //Initialize SPI
    spi1_init();
    
    //data variables
    unsigned short c1,c2 ;
    //unsigned char BUFGASHD = 0b111 ;
    //unsigned char chAB ;
    //unsigned char voltageA = 200;
    //unsigned char voltageB = 64;
    
    //Waveforms A and B...
    unsigned char voltageA[100], voltageB[100];
    char i = 0 ;
    for (i = 0; i<100; ++i) {
        voltageA[i] = (unsigned char) 127 + 127*sin(i/15.915) ;
        if (i<=50) {
            voltageB[i] = (unsigned char) i*(255/50.0) ;
        }
        if (i>50) {
            voltageB[i] = (unsigned char) 255 - i*(255/50.0) ;
        }
        }
    
    //counter
    char count = 0;
    
    __builtin_enable_interrupts();
    
    while(1) {
        
        //Channel A...
        c1 = 0b0111000000000000;
        c1 = c1 | (voltageA[count] << 4);
        
        //Channel B...
        c2 = 0b1111000000000000;
        c2 = c2 | (voltageB[count] << 4);
        
        //sending
        CS = 0;
        spi1_io(c1) ;
        CS = 1;
        
        _CP0_SET_COUNT(0);                       //set core timer to 0
        while(_CP0_GET_COUNT() <= 4000) { ; }   //
        
        CS = 0;
        spi1_io(c2) ;
        CS = 1;
        
        //delay
        _CP0_SET_COUNT(0);                       //set core timer to 0
        while(_CP0_GET_COUNT() <= 100000) { ; }   // delay by 0.125 ms
        
        //counter and reset
        count++ ;
        if (count == 100){
            count = 0;
        }        
    }
}
