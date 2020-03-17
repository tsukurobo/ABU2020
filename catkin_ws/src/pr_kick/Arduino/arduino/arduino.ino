//ise-motor-driver
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
//ros
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
//debug
//#include <std_msgs/Float64.h>

//constant
const uint8_t ADDR = 0x12;  //address of AVR on ise-motor-driver
const int TOUCH_PIN = 6;    //pin
const int SOLENOID_PIN = 7; //pin
const int MAIN_DELAY = 10;   //[millisec]

//global variable
int pw = 0; //power of moter (-255~255)
int order_wind   = 0; //order of wind rope
int order_launch = 0; //order of kick
int delay_time = 0; //delay time of solenoid on/off [milli sec]

//function prottype
void callback(const std_msgs::Int16MultiArray& msg);
void winding();
void launching();

//init
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16MultiArray>  sub("kick_order",&callback);
std_msgs::Int16MultiArray data;
ros::Publisher pub("kick_fin",&data);
//debug
//std_msgs::Float64 debug;
//ros::Publisher pub_d("debug_tpc",&debug);

IseMotorDriver motor(ADDR);
  
void setup(){
  Wire.begin();
  
  //init node
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  //debug
  //nh.advertise(pub_d);

  data.data_length = 3;
  data.data = (int16_t*)malloc(sizeof(int16_t)*3);
  data.data[0] = 0;
  data.data[1] = 0;
  
  
  //init pin
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);
}

void loop(){
  nh.spinOnce();

  //wind
  if(order_wind==1) winding();
  
  //kick
  if(order_launch==1){
    launching();
  }else{
    digitalWrite(SOLENOID_PIN,LOW);
  }
  //debug.data = 2;
  //pub_d.publish(&debug);
  delay(MAIN_DELAY); 
}

//winding
void winding(){
  //value of touch sensor (OFF:LOW ON:HIGH) 
  long enc = 0; //value of encoder [step];

  //normal rotation
  do{
    nh.spinOnce();
    if(order_wind<0) goto RESET;

    motor.setSpeed(pw);
    enc = motor.encorder();
    delay(MAIN_DELAY);
  }while(digitalRead(TOUCH_PIN) == LOW);
    
  motor.setSpeed(0);

  //reverse rotation
  do{
    nh.spinOnce();
    if(order_wind<0) goto RESET;

    motor.setSpeed(-pw);
    enc = motor.encorder();
    delay(MAIN_DELAY);
  }while(enc>0);

  //finish
  data.data[0] = 0;
  pub.publish(&data);
RESET:
  motor.setSpeed(0);
  
}

void launching(){
  //value of touch sensor (OFF:LOW ON:HIGH) 
  bool touch;
  
  nh.spinOnce();
  if(order_launch<0) goto RESET;

  touch = digitalRead(TOUCH_PIN);

  while((order_launch>0) && (touch==HIGH)){
    digitalWrite(SOLENOID_PIN,HIGH);
    delay(delay_time);
    digitalWrite(SOLENOID_PIN,LOW);
    delay(delay_time*9);

    touch = digitalRead(TOUCH_PIN);

    if(digitalRead(TOUCH_PIN)==LOW){
      data.data[1] = 0;
      pub.publish(&data);
    }
    
    nh.spinOnce();
  }
  
RESET:
;
}

//ros callback function
void callback(const std_msgs::Int16MultiArray& msg){
  order_wind   = msg.data[0];
  order_launch = msg.data[1];
  pw           = msg.data[2];
  delay_time   = msg.data[3];
}
