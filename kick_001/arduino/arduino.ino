//ピン配置
//センサはデジタルピンとGNDに繋ぐ

//使い方
//リセットボタンでスタンバイ
//シリアルモニタでデータを入れると正転スタート
//タッチセンサが反応してストップ
//シリアルモニタでデータを入れると逆転スタート
//正転と同じ回転数回転したら自動でストップ
//次使う前にマイコンとモードラのリセットボタンを押す

#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"

/*address of AVR on ise-motor-driver*/
uint8_t addr = 0x12;
/*pin setting*/
const int TOUCH_PIN = 7;
const int SOLENOID_PIN = 9;
/*other constants*/
const int DELAY_TIME = 10;

/*global varialbes*/
int pw = 60; //power of moter (-100~100)
long enc = 0; //value of encoder [step]
int val = 0; //value of bytes available for reading from the serial port [byte]
bool touch = LOW; //value of touch sensor (OFF:LOW ON:HIGH) 

IseMotorDriver motor = IseMotorDriver(addr);

void setup(){
  Wire.begin();
  Serial.begin(115200);
  pinMode(TOUCH_PIN, INPUT_PULLUP);

  //standing by mode
  do{
    motor.setSpeed(0);
    Serial.println("standing by");

    val = Serial.available();
    delay(DELAY_TIME);
  }while(val==0);

  //normal ratation mode
  do{
    motor.setSpeed(pw);
    enc = motor.encorder();
    touch = digitalRead(TOUCH_PIN);
    
    Serial.print("power: "); Serial.print(pw);
    Serial.print('\t');
    Serial.print("encoder: "); Serial.println(enc);

    //val = Serial.available();
    delay(DELAY_TIME);
  }while(touch==LOW);

  //stopping mode
  do{
    motor.setSpeed(0);
    Serial.print("stopping"); Serial.print('\t');
    Serial.print("encoder: "); Serial.println(enc);

    val = Serial.available();
    delay(DELAY_TIME);
  }while(Serial.available()==val);

  //reverse rotation mode
  do{
    motor.setSpeed(-pw);
    enc = motor.encorder();
    
    Serial.print("power: "); Serial.print(-pw);
    Serial.print('\t');
    Serial.print("encoder: "); Serial.println(enc);
    
    delay(DELAY_TIME);
  }while(enc>0);

  //stopping mode
  while(1){
    motor.setSpeed(0);
    Serial.println("finish"); //Serial.print('\t');
    //Serial.print("encoder: "); Serial.println(enc);

    delay(DELAY_TIME);
  }
}

void loop(){
}
