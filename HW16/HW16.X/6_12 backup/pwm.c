//source file for PWM output from OC1 (A0) using  Timer2

#include "pwm.h"
#include <xc.h>
#include <sys/attribs.h>

void pwm_init() {
    
    OC1CONbits.OCTSEL = 1;
    T3CONbits.TCKPS = 0; // Timer3 prescaler N=1 (1:1)
    PR3 = 2399; // PR = PBCLK / N / desiredF - 1
    TMR3 = 0; // initial TMR2 count is 0
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC1RS = 0; // duty cyclE
    OC1R = 0; // initialize before turning OC1 on; afterward it is read-only
    T3CONbits.ON = 1; // turn on Timer3
    OC1CONbits.ON = 1; // turn on OC1

}

void pwm2_init() {
    
    OC4CONbits.OCTSEL = 1;
    T3CONbits.TCKPS = 0; // Timer3 prescaler N=1 (1:1)
    PR3 = 2399; // PR = PBCLK / N / desiredF - 1
    TMR3 = 0; // initial TMR2 count is 0
    OC4CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC4RS = 0; // duty cyclE
    OC4R = 0; // initialize before turning OC1 on; afterward it is read-only
    T3CONbits.ON = 1; // turn on Timer3
    OC4CONbits.ON = 1; // turn on OC4
}

void pwm_active(int on) {

	if (on == 1) {
		OC1CONbits.ON = 1;
		
	}
	if (on == 0) {
		OC1CONbits.ON = 0;

	}
}

void pwm2_active(int on) {

	if (on == 1) {
		OC4CONbits.ON = 1;
		
	}
	if (on == 0) {
		OC4CONbits.ON = 0;

	}
}

void pwm_duty(int duty) {
	
	OC1RS = (duty/100)*(PR3);

}

void pwm2_duty(int duty) {
	
	OC4RS = duty*(PR3)/100;

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
	PR4 = 18749;                    //             set period register 
    T4CONbits.TCKPS = 0b111;        // N = 256
	TMR4 = 0;                       //             initialize count to 0
	T4CONbits.TGATE = 0;            //             not gated input (the default)
	T4CONbits.ON = 1;               //             turn on Timer4
	IPC4bits.T4IP = 5;              // INT step 4: priority
	IPC4bits.T4IS = 0;              //             subpriority
	IFS0bits.T4IF = 0;              // INT step 5: clear interrupt flag
	IEC0bits.T4IE = 1;              // INT step 6: enable interrupt
}
