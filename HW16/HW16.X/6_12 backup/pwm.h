// .h file for the PWM module!!

#ifndef pwm_h
#define pwm_h

#include <xc.h>
#include <sys/attribs.h>

//other stuff...
volatile int percent;

//pwm1...
void pwm_init();
void pwm_active(int on);
void pwm_duty(int duty);
void pwm_command(int percent);

//pwm2...
void pwm2_init();
void pwm2_active(int on);
void pwm2_duty(int duty);

//interrupt...
void interrupt_init();

#endif