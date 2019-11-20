/*
 * TRCE.cpp
 *
 * Created: 2016/09/17 12:17:09
 * Author : kazuma
 */ 

#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "trctwi.hpp"

volatile long count;

ISR(PCINT0_vect)
{
	if((PINB & (1 << PINB0)) ^ (PIND & (1 << PIND0)))--count;
	else ++count;
	
}

void init()
{
	count = 0;
	
	DDRC = 0x00;
	PORTC = 0x00;
	
	DDRD = 0x00;
	PORTD = 0x00;
	
	//ピン変化割り込み許可（PCINT0~7）
	PCICR |= (1<<PCIE0);
	//ピン変化割り込み許可（PCINT0）
	PCMSK0 |= (1<<PCINT0);
}


int main(void)
{
    init();
    sei();
	TRCTWI tr1;
	tr1.initSlaveSender(0x10);
    while (1)
    {
	    tr1.slaveSend(count, 8);
	}
}
