#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<string.h>
#include "i2c.h"
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
    i2c_master_setup() ;
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_GREENYELLOW);
    unsigned char a = 234;
    unsigned short x1 = 2;
    unsigned short y1 = 300;
    unsigned short color = ILI9341_WHITE ;
    unsigned short color_bgd = ILI9341_BLACK ;
    
    //heartbeat...
    TRISAbits.TRISA4 = 0 ;
    LATAbits.LATA4 = 0 ; 
    
    //Chip address...
    const char ADDRESS = 0b1101011;
    
    //registers.....
    const char WHO_AM_I = 0x0F;
    const char CTRL1_XL = 0x10;
    const char CTRL2_G = 0x11;
    const char CTRL3_C = 0x12;
    const char OUT_TEMP_L = 0x20;
    const char OUTX_L_XL = 0x28;
    
    //reading who_am_i
    unsigned char who_am_i;
    unsigned char read;
    who_am_i = i2c_master_read(ADDRESS,WHO_AM_I) ;
    
    //initializing the chip...
    i2c_master_write(ADDRESS, CTRL1_XL, 0x82); //To the accelerometer
    i2c_master_write(ADDRESS, CTRL2_G, 0x88); //To the gyroscope
    i2c_master_write(ADDRESS, CTRL3_C, 0x04); //Turning on IF_INC
    
    //multiple reads...
    int LENGTH = 14;
    unsigned char data[LENGTH];
    unsigned char msg0[20];
    unsigned char msg1[20];

    //test       
    unsigned char print[20];
    sprintf(print,"mark is the best");
    
    //data shorts...
    signed short gyro_x = 0;
    signed short gyro_x_scaled;
    signed short gyro_y = 0;
    signed short gyro_y_scaled;
    signed short gposn_x = 0;
    signed short gposn_x_old = 0;
    signed short gposn_x_scaled;
    signed short gposn_y = 0;
    signed short gposn_y_old = 0;
    signed short gposn_y_scaled;
    
    //box bar...
    unsigned short xlen = 240;
    unsigned short ylen = 10;
    unsigned short boxpos;
    unsigned char boxhalf = 5;
    
    __builtin_enable_interrupts();

    while(1) {
       //heartbeat (ON)
       LATAbits.LATA4 = 1 ; 
       
       //reading acceleration......
       read = i2c_master_read(ADDRESS,0x24);
       LCD_bytebin(read,x1,y1,color,color_bgd); 
       
       //multiple reads...
       i2c_read_multiple(ADDRESS,OUT_TEMP_L,data,LENGTH);
      
       //heartbeat (off)
       LATAbits.LATA4 = 0 ; 
       
       //gyro values...
       gyro_x = (data[3]<<8) | data[2];
       gyro_y = (data[5]<<8) | data[4];
       
       //scaling...
       gyro_x_scaled = (signed short) gyro_x/20;
       gyro_y_scaled = (signed short) gyro_y/20+12;
       
       //position values...
       if((gyro_y_scaled > 3) | (gyro_y_scaled < -3)){
            gposn_y = gposn_y_old + gyro_y_scaled;
       }
       if((gyro_x_scaled > 3) | (gyro_x_scaled < -3)){
            gposn_x = gposn_x_old + gyro_x_scaled;
       }
       
       //box bars
       LCD_boxbar(0,0,160,xlen,ylen,gposn_y+125,boxhalf,color,ILI9341_DARKGREEN);
       LCD_boxbar(1,115,0,320,ylen,165-gposn_x,boxhalf,color,ILI9341_DARKGREEN);
       
       //prints
       sprintf(msg0,"gyro_x is %4d",gyro_x_scaled);
       LCD_print(msg0,0,0,color,color_bgd);
       sprintf(msg0,"gyro_y is %4d",gyro_y_scaled);
       LCD_print(msg0,0,10,color,color_bgd);
       
       gposn_x_old = gposn_x;
       gposn_y_old = gposn_y;
        
        
    }
}



/*
void initIMU() {
    char address = 0b0100111;
    char regstr = 0x00;
    char lat = 0x0A;
    
    //setting GP0-GP3 to OUTPUT and GP4-GP7 to INPUT
    char value = 0b11110000;
    i2c_master_write(address, regstr, value); //sending
}
 */
