#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ili9341.h"
#include "pwm.h"

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

    //Initialize...
    ANSELA = 0;
    ANSELB = 0;
    
    //LCD...
    unsigned short x1 = 2;
    unsigned short y1 = 20;
    unsigned short xorigin = 1;
    unsigned short yorigin = 120;
    unsigned short xlen = 230;
    unsigned short ylen = 30;
    
    unsigned short color = ILI9341_WHITE ;
    unsigned short color_bgd = ILI9341_BLACK ;
    char message[40];
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_WHITE);
    
    //making the data array
    unsigned char data[3][240];
    unsigned char i, j;
    for(i = 0; i<240; ++i){
        for(j = 0; j<3 ; ++j){
            data[j][i] = 9*(i-7*j);
        }
    } 
    
    //test print...
    sprintf(message,"poop poop %d",data[2][38]);
    LCD_print(message,x1,y1,color,color_bgd);

    
    //Drawing out the axes
    
    LCD_plot(data[0],xorigin,yorigin,xlen,ylen,ILI9341_RED);
    LCD_plot(data[1],xorigin,yorigin+50,xlen,ylen,ILI9341_DARKGREEN);
    LCD_plot(data[2],xorigin,yorigin+100,xlen,ylen,ILI9341_BLUE);
    
    //output compare...
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA4 = 0;
    RPA0Rbits.RPA0R = 0b0101; //mapping A0 to OC1...
    pwm_init();
    pwm_active(1);
    int duty = 50;
    pwm_duty(duty);
    
    interrupt_init();
    
    __builtin_enable_interrupts();
    
    

    while(1) {
        // 
    }
}
