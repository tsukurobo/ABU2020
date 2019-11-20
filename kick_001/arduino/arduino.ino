//使い方
//リセットボタンでスタンバイ
//シリアルモニタでデータを入れると正転スタート
//タッチセンサが反応してストップ
//シリアルモニタでデータを入れると逆転スタート
//正転と同じ回転数回転したら自動でストップ
//次使え前にリセットボタンを押す

#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"

/*address of AVR on ise-motor-driver*/
uint8_t addr = 0x12;
/*pin setting*/
const int TOUCH_PIN1 = 7;
const int TOUCH_PIN2 = 6;

/*global varialbes*/
int pw = 60; //power of moter (-100~100)
long enc = 0; //value of encoder [step]
long encMax = 0;
int val = 0; //value of bytes available for reading from the serial port [byte]
bool touch = HIGH; //value of touch sensor (OFF:HIGH ON:LOW) 

IseMotorDriver motor = IseMotorDriver(addr);

void setup(){
  Wire.begin();
  Serial.begin(9600);
  pinMode(TOUCH_PIN1, OUTPUT);
  pinMode(TOUCH_PIN2, INPUT);

  //standing by mode
  while(val==0){
    motor.setSpeed(0);
    Serial.println("standing by");
    
    val = Serial.available();
    delay(10);
  }

  //normal ratation mode
  while(touch==HIGH){
    motor.setSpeed(pw);
    enc = motor.encorder();
    digitalWrite(TOUCH_PIN1,HIGH);
    touch = digitalRead(TOUCH_PIN2);
    
    Serial.print("power: "); Serial.print(pw);
    Serial.print('\t');
    Serial.print("encoder: "); Serial.println(enc);

    val = Serial.available();
    delay(10);
  }

  //stopping mode
  while(Serial.available()==val){
    motor.setSpeed(0);
    Serial.print("stopping"); Serial.print('\t');
    Serial.print("encoder: "); Serial.println(enc);

    val = Serial.available();
    delay(10);
  }

  //reverse rotation mode
  while(enc>0){
    motor.setSpeed(-pw);
    enc = motor.encorder();
    Serial.print("power: "); Serial.print(-pw);
    Serial.print('\t');
    Serial.print("encoder: "); Serial.println(enc);
  }

  //stopping mode
  while(1){
    motor.setSpeed(0);
    Serial.println("stopping"); //Serial.print('\t');
    //Serial.print("encoder: "); Serial.println(enc);

    delay(10);
  }
}

void loop(){
}
