#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

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

//Function declarations...
void initExpander();
void setExpander(char pin, unsigned char level);
unsigned char getExpander();

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

    //Initialize the I2C2 pins B2 and B3...
    ANSELBbits.ANSB2 = 0; //Setting B2 and B3 to 
    ANSELBbits.ANSB3 = 0;
    
    //Initializing I2C2...
    i2c_master_setup() ;
    
    //HW1 stuff for testing
    TRISAbits.TRISA4 = 0;   //LED Pin A4 set to output
    TRISBbits.TRISB4 = 1;   //USER button pin B4 set to input
    LATAbits.LATA4 = 0;     //LED Set to LOW  
    
    //Initializing...
    initExpander();
    
    __builtin_enable_interrupts();
        
    while(1) {
        
        //testing the pin expander
        char pin = 0;
        unsigned char lev;
        
        //unsigned char read = getExpander();
        //setread(read>>4);
        
        //if(PORTBbits.RB4 == 0){    
        //    setExpander(pin,lev);
        //    LATAbits.LATA4 = 1;
        //}
        
        //getting GP0 to work...
        while(PORTBbits.RB4 == 0) {
            LATAbits.LATA4 = 1 ;
            lev = 1;
            setExpander(pin,lev);
            while(_CP0_GET_COUNT() <= 60000000) { ; }  
            LATAbits.LATA4 = 0 ;
            lev = 0;
            setExpander(pin,lev);
        }
        
    }   
}

void initExpander() {
    //Chip address = 0b111. The direction register has the address 0x00, LAT is 0x0A
    char address = 0b0100111;
    char direction = 0x00;
    char lat = 0x0A;
    
    //setting GP0-GP3 to OUTPUT and GP4-GP7 to INPUT
    char value = 0b11110000;
    i2c_master_write(address, direction, value); //sending
    
    //setting GP0-GP3 HIGH
    value = 0b00001100;
    i2c_master_write(address, lat, value); //sending
}

void setExpander(char pin, unsigned char level) {
    char address = 0b0100111;
    char lat = 0x0A;
    char value = (level<<pin | 0b00000000);
    i2c_master_write(address, lat, value); //sending
}

unsigned char getExpander() {
    char port = 0x09;
    char address = 0b0100111;
    unsigned char read = i2c_master_read(address,port);
    return(read);
}