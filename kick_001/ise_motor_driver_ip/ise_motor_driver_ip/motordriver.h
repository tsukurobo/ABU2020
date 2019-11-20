/*
 * motordriver.h
 *
 * Created: 2018/01/21 19:37:23
 *  Author: magon
 */ 

void motor_init(void);
void motor_set_speed(int power);
void pwm_map(int duty);