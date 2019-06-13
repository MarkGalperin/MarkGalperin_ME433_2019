/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "i2c.h"
#include "ili9341.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            //My initializations...
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

            TRISAbits.TRISA4 = 0;   //LED Pin A4 set to output
            TRISBbits.TRISB4 = 1;   //USER button pin B4 set to input
            LATAbits.LATA4 = 0;     //LED Set to LOW  

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
                
        
            if (appInitialized)
            {
                
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
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
            
            //HW1 CODE
            /*
            while(PORTBbits.RB4 == 0) {
                LATAbits.LATA4 = 1 ;                 //LED Set to HIGH

                _CP0_SET_COUNT(0);                       //set core timer to 0
                while(_CP0_GET_COUNT() <= 12000) { ; }   // delay by 0.5 ms

                LATAbits.LATA4 = 0;                     //LED Set to HIGH

                _CP0_SET_COUNT(0);                       //set core timer to 0
                while(_CP0_GET_COUNT() <= 12000) { ; }   // delay by 0.5 ms again
            }
            */
            
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
