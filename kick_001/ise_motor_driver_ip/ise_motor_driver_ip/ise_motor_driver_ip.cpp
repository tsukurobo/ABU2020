/*
 * ise_motor_driver.cpp
 *
 * Created: 2018/03/09 19:51:04
 * Author : Yoshihara(kari)
 */ 
#define F_CPU 20000000UL

#include <avr/io.h>
#include "I2CSlave.h"
#include "TI2C.h"
#include "motordriver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define I2C_ADDR 0x40


volatile long count = 0;
volatile int pw = 0;

void i2c_received_cb(char* str) {
	motor_set_speed(atoi(str));
}

void i2c_request_cb(char* buf) {
	// set TI2C_buf_send.str_buf
	//strcpy(buf, "Hello World\n");
	sprintf(buf,"%ld",count);
}

ISR(PCINT1_vect, ISR_NOBLOCK){//encorder
	if((PINC & (1 << PINC0)) ^ ((PINC & (1 << PINC1))>> 1))--count;
	else ++count;	
}


void setup (){
	
	//ピン変化割り込み許可（PCINT8~15）
	PCICR |= (1<<PCIE1);
	//ピン変化割り込み許可（PCINT8）
	PCMSK1 |= (1<<PCINT8);
	motor_init();
	motor_set_speed(pw);
	
	DDRC = 0x00;
	PORTC = 0x00;
	
	// LED of addresses 0x10~ 0x1F 0~F -> 0~15
	PORTD |= (0b00001111 & I2C_ADDR);

	//sei();
	
	TI2C_init_sync(I2C_ADDR,i2c_received_cb, i2c_request_cb);
}


int main(void)
{
	setup();
	
    /* Replace with your application code */
    while (1) 
    {
		I2C_main();
    }
}

