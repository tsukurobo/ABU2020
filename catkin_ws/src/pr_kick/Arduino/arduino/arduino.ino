//ise-motor-driver
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
//ros
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
//debug
#include <std_msgs/Float64.h>

//constant
const uint8_t ADDR = 0x14;  //address of AVR on ise-motor-driver
const int TOUCH_PIN = 6;    //pin
const int SOLENOID_PIN = 7; //pin
const int MAIN_DELAY = 10;   //[millisec]

//global variable
int pw; //power of moter (-100~100)
int order_wind = 0; //order of wind rope
int order_kick = 0; //order of kick

//function prottype
void callback(const std_msgs::Int16MultiArray& msg);
void winding();

//init
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16MultiArray>  sub("kick_tpc",&callback);
//debug
std_msgs::Float64 data;
ros::Publisher pub("debug_tpc",&data);

IseMotorDriver motor(ADDR);
  
void setup(){
  Wire.begin();
  
  //init node
  nh.initNode();
  nh.subscribe(sub);
  //debug
  nh.advertise(pub);

  //init pin
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  //init power
  while(pw==0){
    nh.spinOnce();
    delay(MAIN_DELAY);
  }
  

}

void loop(){
  nh.spinOnce();

  //wind
  if(order_wind==1) winding();
  
  //kick
  if(order_kick==1){
    digitalWrite(SOLENOID_PIN, HIGH);
  }else{
    digitalWrite(SOLENOID_PIN, LOW);
  }
  
  delay(MAIN_DELAY); 
}

//winding
void winding(){
  //value of touch sensor (OFF:LOW ON:HIGH) 
  long enc = 0; //value of encoder [step];
    
  //normal rotation
  do{
    nh.spinOnce();
    motor.setSpeed(pw);
    enc = motor.encorder();
    delay(MAIN_DELAY);
  }while(digitalRead(TOUCH_PIN) == LOW);

  motor.setSpeed(0);
    
  //reverse rotation
  do{
    nh.spinOnce();
    motor.setSpeed(-pw);
    enc = motor.encorder();
    delay(MAIN_DELAY);
  }while(enc<0);

  //finish
  motor.setSpeed(0);


}

//ros callback function
void callback(const std_msgs::Int16MultiArray& msg){
  order_wind   = msg.data[0];
  order_kick   = msg.data[1];
  pw           = msg.data[2];
}
