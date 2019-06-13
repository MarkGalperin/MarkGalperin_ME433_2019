//source file for PWM output from OC1 (A0) using  Timer2

#include "pwm.h"
#include <xc.h>
#include <sys/attribs.h>



//First, the ISR...
void __ISR(12, IPL5SOFT) Timer3ISR(void) {
    
    static unsigned int isr_count = 0;
    static unsigned char direction = 1;
    
    
    if (direction == 1){
        isr_count++;
        LATAbits.LATA4 = 1;
        if (isr_count>1000){
            direction = 0;
        }
    }
    if (direction == 0){
        isr_count--;
        LATAbits.LATA4 = 0;
        if (isr_count<2){
            direction = 1;
        }
    }
    
    
    // set the duty cycle and direction pin
    OC1RS = (unsigned int) ((isr_count/1000.0) * PR2);
    
    IFS0bits.T3IF = 0;
}


void pwm_init() {
    
    T2CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
    PR2 = 2399; // PR = PBCLK / N / desiredF - 1
    TMR2 = 0; // initial TMR2 count is 0
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC1RS = 0; // duty cyclE
    OC1R = 0; // initialize before turning OC1 on; afterward it is read-only
    T2CONbits.ON = 1; // turn on Timer2
    OC1CONbits.ON = 1; // turn on OC1

}

void pwm_active(int on) {

	if (on == 1) {
		OC1CONbits.ON = 1;
		
	}
	if (on == 0) {
		OC1CONbits.ON = 0;

	}
}


void pwm_duty(int duty) {
	
	OC1RS = duty*(PR2)/100;

}

void pwm_command(int percent) {

	//set duty
	if (percent<0) {
		percent = -1*percent;
		pwm_duty(percent);
	}
	if(percent>100) {
		pwm_duty(100);
	}

	//activate
	pwm_active(1);
}

void interrupt_init() {
                                    // INT step 3: setup peripheral
	PR3 = 47999;                    //             set period register   
	TMR3 = 0;                       //             initialize count to 0
	T3CONbits.TCKPS = 0;            //             set prescaler to 1
	T3CONbits.TGATE = 0;            //             not gated input (the default)
	T3CONbits.ON = 1;               //             turn on Timer3
	IPC3bits.T3IP = 5;              // INT step 4: priority
	IPC3bits.T3IS = 0;              //             subpriority
	IFS0bits.T3IF = 0;              // INT step 5: clear interrupt flag
	IEC0bits.T3IE = 1;              // INT step 6: enable interrupt
}