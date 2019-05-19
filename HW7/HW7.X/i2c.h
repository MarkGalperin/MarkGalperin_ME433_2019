//.h file for the I2C2 module!
#ifndef i2c_h
#define i2c_h

#include <xc.h>

//my functions...
void i2c_master_write(char addr,char rgstr,char value);

unsigned char i2c_master_read(char addr,char rgstr) ;

void i2c_read_multiple(unsigned char address, unsigned char register, unsigned char * data, int length);

void i2c_read_multiple2(unsigned char address, unsigned char register, unsigned char * data, int length);


//Nick's functions...
void i2c_master_setup(void) ;

void i2c_master_start(void) ;

void i2c_master_restart(void);

void i2c_master_send(unsigned char byte) ;

unsigned char i2c_master_recv(void) ;

void i2c_master_ack(int val) ;

void i2c_master_stop(void) ;

#endif