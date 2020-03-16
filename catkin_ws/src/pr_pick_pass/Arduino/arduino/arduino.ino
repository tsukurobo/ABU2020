#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
//debug
#include <std_msgs/Float64.h>

//constant
const uint8_t ADDR_PICK = 0x12; //pick up motor address of AVR
const uint8_t ADDR_PASS = 0x14; //pass motor address of AVR
const int TOUCH_PIN    = 5; //touch sensor pin
const int SOLENOID_PIN = 6; //solenoid pin
const int VALVE_PIN_1  = 7; //valve pin
const int VALVE_PIN_2  = 8; //valve pin
const int ENC_PER_ROT = 4048; //[enc_step/360deg]
const int MAIN_DELAY  = 10;   //[millisec]

//global variable
int order_wind   = 0; //order of wind rope
int order_pick   = 0; //order of pick up ball
int order_launch = 0; //order of launch ball
int pw_wind;  //motor power of wind rope (-255~255)
int pw_lower = 40; //motor power of lower hand (-255~255)
int pw_raise = 60; //motor power of raise ball (-255~255)
int target_deg = 120; //degree of lower hand for pick up[deg]

//function prottype
void callback(const std_msgs::Int16MultiArray& msg);
void picking_up(); //watch out enc condition///////////////////////////////////
void winding(); //watch out enc condition//////////////////////////////////////

//init
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16MultiArray>  sub("pp_tpc",&callback);
IseMotorDriver mot_pick(ADDR_PICK);
IseMotorDriver mot_pass(ADDR_PASS);

//debug
std_msgs::Float64 data;
ros::Publisher pub("debug_tpc",&data);
  
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
  pinMode(VALVE_PIN_1, OUTPUT);
  pinMode(VALVE_PIN_2, OUTPUT);
  //init actuator
  digitalWrite(SOLENOID_PIN, LOW);
  digitalWrite(VALVE_PIN_1, LOW);
  digitalWrite(VALVE_PIN_2, LOW);
  mot_pick.setSpeed(0);
  mot_pass.setSpeed(0);
}

void loop(){
  nh.spinOnce();
    
  //wind rope
  if(order_wind==1) winding();
  
  //pick up ball
  if(order_pick==1) picking_up();
  
  //launch ball
  if(order_launch==1){
    //launch ball
    digitalWrite(SOLENOID_PIN, HIGH);
    //reset valve
    digitalWrite(VALVE_PIN_1, LOW);
    digitalWrite(VALVE_PIN_2, LOW);
  }else{
    digitalWrite(SOLENOID_PIN, LOW);
  }
    
  delay(MAIN_DELAY); 
}

//pick up
void picking_up(){
  long enc = 0; //value of encoder [step];

  //open hand
  digitalWrite(VALVE_PIN_1, HIGH);
  digitalWrite(VALVE_PIN_2, LOW);
  
  //lower hand
  do{
    nh.spinOnce();
    mot_pick.setSpeed(pw_lower);
    enc = mot_pick.encorder();
    delay(MAIN_DELAY);
  }while(target_deg > enc);
  mot_pick.setSpeed(0);

  //hold hand
  digitalWrite(VALVE_PIN_1, LOW);
  digitalWrite(VALVE_PIN_2, HIGH);
    
  //raise hand
  do{
    nh.spinOnce();
    mot_pick.setSpeed(pw_raise);
    enc = mot_pick.encorder();
    delay(MAIN_DELAY);

  }while(enc > 0);
  mot_pick.setSpeed(0);

  //open hand
  digitalWrite(VALVE_PIN_1, HIGH);
  digitalWrite(VALVE_PIN_2, LOW);
}

//winding////////////////////////////////////////////////////////////
void winding(){
  //value of touch sensor (OFF:LOW ON:HIGH) 
  long enc = 0; //value of encoder [step];

  //normal rotation
  do{
    nh.spinOnce();
    mot_pass.setSpeed(pw_wind);
    enc = mot_pass.encorder();
    delay(MAIN_DELAY);
  }while(digitalRead(TOUCH_PIN) == LOW);
  
  //reverse rotation
  do{
    nh.spinOnce();
    mot_pass.setSpeed(-pw_wind);
    enc = mot_pass.encorder();
    delay(MAIN_DELAY);
  }while(enc>0);

  //finish
  mot_pass.setSpeed(0);

}

//ros callback function////////////////////////////////////////////////
void callback(const std_msgs::Int16MultiArray& msg){
  order_wind   = msg.data[0];
  order_pick   = msg.data[1];
  order_launch = msg.data[2];
  pw_wind      = msg.data[3];
  pw_lower     = msg.data[4];
  pw_raise     = msg.data[5];
  target_deg   = msg.data[6];
}
