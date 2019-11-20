/*
 * ContorolMotor.cpp
 *
 * Created: 2018/01/21 19:11:22
 *  Author: magon
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
//20khz
//10khz
//1khz
const int TOP=255;
enum{
	TMR0Full = TOP,//255,
	TMR0INI = 0, //122,
};

int motor_init(void)
{
	//setup
	DDRD |= 0b11100000;
	
	TCCR0A = 0b10100011;  //highspeedPWM
	
	TCCR0B = 0b00000010;  //1/8,Top=OVF
}

int pwm_map (int duty){
	return TMR0INI + (int)(TOP/100*duty); //duty=0~100
}
	
	
	
int motor_set_speed (int power) {
	
     	OCR0A = pwm_map(abs(power));
	    OCR0B = pwm_map(abs(power));

		if (power < 0){
			//PORTD = 0b01100000;	  //PHASE=high
			PORTD &= 0b01111111;
		}
		
		else if(power > 0){
			//PORTD = 0b11100000;   //PHASE=low
			PORTD |= 0b10000000;
		}
				
}

