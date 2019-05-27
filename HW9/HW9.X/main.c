#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ili9341.h"

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

//chip select
#define CS_touch LATBbits.LATB9 //B9 is the chip select pin for the touch screen

//definition
//void XPT2046_read(unsigned char* addresses, unsigned short *p);
void XPT2046_read(unsigned short *x,unsigned short *y,unsigned short *z);

//LCD...
unsigned short x1 = 2;
unsigned short y1 = 300;
unsigned short color = ILI9341_WHITE ;
unsigned short color_bgd = ILI9341_BLACK ;
char message[40];

//button boundaries
unsigned short b1_x1 = 150;
unsigned short b1_x2 = 200;
unsigned short b1_y1 = 20;
unsigned short b1_y2 = 60;

unsigned short b2_x1 = 150;
unsigned short b2_x2 = 200;
unsigned short b2_y1 = 80;
unsigned short b2_y2 = 120;

//control byte...
unsigned char command; 
unsigned char garbage;
unsigned char val1;
unsigned char val2;
unsigned char i = 0;
signed char j = 0;
unsigned char addresses[4] = {0b001,0b011,0b100,0b101};
unsigned short readval[4];

//x,y,z
unsigned short x_t;
unsigned short y_t;
unsigned short z_t;
unsigned short x_old = 0;
unsigned short y_old = 0;
unsigned short z_old = 0;
unsigned short x_pix;
unsigned short y_pix;
unsigned short p;
unsigned short *px = &x_t;
unsigned short *py = &y_t;
unsigned short *pz = &z_t;


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

    //Chip select output...
    TRISBbits.TRISB9 = 0 ;  //B9 set to output 
    
    //Initialize LCD and test print...
    ANSELA = 0;
    ANSELB = 0;
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_CYAN);
    
    //heartbeat...
    TRISAbits.TRISA4 = 0 ;
    LATAbits.LATA4 = 0 ; 
    
    __builtin_enable_interrupts();

    while(1) {
        
        LATAbits.LATA4 = 1 ; //heartbeat on
        
        XPT2046_read(px,py,pz);
        
        //pixel values...
        x_pix = (unsigned short) (x_t-2500)/(118.75);
        y_pix = (unsigned short) (y_t-2500)/(87.5);
        
        //drawing the boxes...
        LCD_button(b1_x1 , b1_y1 , b1_x2-b1_x1 , b1_y2-b1_y1 ,ILI9341_DARKGREY,color,color_bgd);
        LCD_button(b2_x1 , b2_y1 , b2_x2-b2_x1 , b2_y2-b2_y1 ,ILI9341_DARKGREY,color,color_bgd);
        
        LATAbits.LATA4 = 0 ; //heartbeat off
        
        //checking for presses on the buttons...
        if(z_t < 10000 && z_old > 10000){
                
            if(x_old > b1_x1 && x_old < b1_x2){
                if(y_old > b1_y1 && y_old < b1_y2){
                    j++;   
                } 
            }
        
            if(x_old > b2_x1 && x_old < b2_x2){
                if(y_old > b2_y1 && y_old < b2_y2){
                    j--;   
                } 
            }                
        }
        
        x_old = x_pix;
        y_old = y_pix;
        z_old = z_t;
        
        //printing raw values
        sprintf(message,"x: %5d",x_t);
        LCD_print(message,x1,10,color,color_bgd);
        
        sprintf(message,"y: %5d",y_t);
        LCD_print(message,x1,30,color,color_bgd);
        
        sprintf(message,"z: %5d",z_t);
        LCD_print(message,x1,50,color,color_bgd);
        
        
        //printing pixel values...
        sprintf(message,"(%3d,%3d)",x_pix,y_pix);
        LCD_print(message,x1,70,color,color_bgd);
        
        //j value...
        sprintf(message,"j = %4d",j);
        LCD_print(message,b1_x1,b1_y2+7,color,color_bgd);
        
        //button labels...
        sprintf(message,"j+");
        LCD_print(message,b1_x2+5,b1_y1+15,color,color_bgd);
        
        sprintf(message,"j-");
        LCD_print(message,b2_x2+5,b2_y1+15,color,color_bgd);
        
    }
}

void XPT2046_read(unsigned short *x,unsigned short *y,unsigned short *z){
    
    for(i = 0; i<4; i++){
        //making the command byte...
        command = 0b10000001 | (addresses[i]<<4);
        
        //sending...
        CS_touch = 0;
        garbage = spi_io(command);
        val1 = spi_io(0x00);
        val2 = spi_io(0x00);
        CS_touch = 1;

        //Making the short...
        readval[i] = (val1<<8) | val2;   
    }
    
    *px = readval[3];
    *py = 32760-readval[0];
    *pz = readval[1]-readval[2]+4095-32760;
}
